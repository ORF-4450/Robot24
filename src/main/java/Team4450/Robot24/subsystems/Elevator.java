package Team4450.Robot24.subsystems;

import static Team4450.Robot24.Constants.ELEVATOR_MOTOR_LEFT;
import static Team4450.Robot24.Constants.ELEVATOR_MOTOR_RIGHT;
import static Team4450.Robot24.Constants.ELEVATOR_WINCH_FACTOR;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import Team4450.Lib.Util;
import Team4450.Robot24.AdvantageScope;
import Team4450.Robot24.Robot;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Elevator subsystem for the 2024 robot USS ProtoStar.
 * Should not be used on its own, should be contained within ElevatedShooter subsystem for safety.
 */
public class Elevator extends SubsystemBase {
    private CANSparkMax motorMain = new CANSparkMax(ELEVATOR_MOTOR_LEFT, MotorType.kBrushless);
    private CANSparkMax motorFollower = new CANSparkMax(ELEVATOR_MOTOR_RIGHT, MotorType.kBrushless);

    // we use a ProfiledPIDController for acceleration and deceleration control
    private ProfiledPIDController mainPID;

    // not really using these... womp womp
    private SparkLimitSwitch lowerLimitSwitch;
    private SparkLimitSwitch upperLimitSwitch;
  
    private RelativeEncoder mainEncoder;
    private RelativeEncoder followEncoder;

    private final double TOLERANCE_COUNTS = 1.5; // in encoder counts, not "meters"
    private final double START_COUNTS = 0.08 / ELEVATOR_WINCH_FACTOR; // the start counts

    private double goal = Double.NaN;

    public Elevator() {
        Util.consoleLog();

        // follower is mirrored and reversed
        // don't change this it's very important as shafts are linked with coupler
        // and will shatter if driven in opposite directions
        motorFollower.follow(motorMain, true);
        motorFollower.setInverted(true);

        motorFollower.setIdleMode(IdleMode.kBrake);
        motorMain.setIdleMode(IdleMode.kBrake);

        // we aren't using these...
        lowerLimitSwitch = motorFollower.getReverseLimitSwitch(Type.kNormallyOpen);
        upperLimitSwitch = motorFollower.getForwardLimitSwitch(Type.kNormallyOpen);
        lowerLimitSwitch.enableLimitSwitch(true);
        upperLimitSwitch.enableLimitSwitch(true);

        mainEncoder = motorMain.getEncoder();
        followEncoder = motorFollower.getEncoder();

        resetEncoders();

        // PID constants, but also the motion profiling constraints
        mainPID = new ProfiledPIDController(0.12, 0, 0, new Constraints(
            (1 / -ELEVATOR_WINCH_FACTOR), 8 / -ELEVATOR_WINCH_FACTOR // velocity / acceleration
        ));
        
        SmartDashboard.putData("winch_pid", mainPID);
        mainPID.setTolerance(TOLERANCE_COUNTS);
    }

    // for closed loop control on Spark MAXs: never worked
    // private void configurePID(SparkPIDController pidController, double p, double i, double d) {
    //     pidController.setP(p);
    //     pidController.setI(i);
    //     pidController.setD(d);
    // }

    @Override
    public void periodic() {
        // to reset using limit switchs... which we don't have...
        // if (lowerLimitSwitch.isPressed()) {
        //     mainEncoder.setPosition(0); // reset the encoder counts
        //     followEncoder.setPosition(0);
        // }
        
        // simulation and logging
        AdvantageScope.getInstance().setElevatorHeight(getElevatorHeight());
        SmartDashboard.putNumber("winch_measured", mainEncoder.getPosition());
        SmartDashboard.putNumber("winch_1_m", mainEncoder.getPosition() * ELEVATOR_WINCH_FACTOR);
        SmartDashboard.putNumber("winch_2_m", followEncoder.getPosition() * ELEVATOR_WINCH_FACTOR);
        SmartDashboard.putNumber("winch_setpoint", goal);

        if (Double.isNaN(goal)) return; // in "unlocked"/"limp" mode so just return

        // SOFT LIMITS ================
        if (goal < -59)
            goal = -59;
        if (goal > -5)
            goal = -5;

        // main pid/profile loop
        mainPID.setGoal(goal);
        double nonclamped = mainPID.calculate(mainEncoder.getPosition());
        double motorOutput = Util.clampValue(nonclamped, 1);
        SmartDashboard.putNumber("elevator speed", motorOutput);
        motorMain.set(motorOutput);

        // output logging and simulation
        SmartDashboard.putNumber("winch_output", motorOutput);
        
        if (Robot.isSimulation()) mainEncoder.setPosition(mainEncoder.getPosition() + (1*motorOutput));
        if (Robot.isSimulation()) followEncoder.setPosition(followEncoder.getPosition() + (1*motorOutput));
    }

    /**
     * remove setpoint generation, essentially making the elevator go limp
     * and disables normal move() commands.
     */
    public void unlockPosition() {
        goal = Double.NaN;
    }

    /**
     * increment/decrement the setpoint/goal value
     * @param change (such as from a joystick value)
     */
    public void move(double change) {
        goal -= change;
    }

    /**
     * Bypass all setpoint generation and just run direct motor power. This
     * also bypasses all soft limits, so use EXTREME CAUTION! Remember, the elevator
     * is capable of TEARING OFF LIMBS AND DESTROYING ITSELF with a force of MORE THAN
     * 400 POUNDS.
     * @param speed the direct motor speed
     */
    public void moveUnsafe(double speed) {
        goal = Double.NaN;
        motorMain.set(speed);
    }

    /**
     * set the elevator setpoint/goal to the given height
     * @param height in "meters"
     */
    public void setElevatorHeight(double height) {
        goal = height / ELEVATOR_WINCH_FACTOR; // meters -> enc. counts
    }

    /**
     * check if the elevator is at a given height within tolerance
     * @param height the height in "meters" to check against
     * @return whether it's close enough (within tolerance)
     */
    public boolean elevatorIsAtHeight(double height) {
        double setpoint = height / ELEVATOR_WINCH_FACTOR; // "meters" to counts
        return Math.abs(setpoint - mainEncoder.getPosition()) < TOLERANCE_COUNTS;
    }

    /**
     * Get the height in "meters" of the elevator/pivot.
     * @return height in "meters"
     */
    public double getElevatorHeight() {
        // encoder counts to "meters"
        double mainValue = mainEncoder.getPosition() * ELEVATOR_WINCH_FACTOR;
        return mainValue;
    }

    /**
     * reset the ecoders to the current position, essentially says wherever it is
     * is the "zero"/startup position
     */
    public void resetEncoders() {
        mainEncoder.setPosition(START_COUNTS);
        followEncoder.setPosition(START_COUNTS);
        goal = START_COUNTS;
    }

    /**
     * set the setpoint to the current position, essentially "locking" the
     * mechanism in place
     */
    public void lockPosition() {
        goal = mainEncoder.getPosition();
        mainPID.reset(goal);
    }
}
