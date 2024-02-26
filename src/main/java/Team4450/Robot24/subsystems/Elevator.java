package Team4450.Robot24.subsystems;

import static Team4450.Robot24.Constants.ELEVATOR_CENTERSTAGE_FACTOR;
import static Team4450.Robot24.Constants.ELEVATOR_MOTOR_INNER;
import static Team4450.Robot24.Constants.ELEVATOR_MOTOR_LEFT;
import static Team4450.Robot24.Constants.ELEVATOR_MOTOR_RIGHT;
import static Team4450.Robot24.Constants.ELEVATOR_WINCH_FACTOR;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import Team4450.Lib.Util;
import Team4450.Robot24.AdvantageScope;
import Team4450.Robot24.Robot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private CANSparkMax motorMain = new CANSparkMax(ELEVATOR_MOTOR_LEFT, MotorType.kBrushless);
    private CANSparkMax motorFollower = new CANSparkMax(ELEVATOR_MOTOR_RIGHT, MotorType.kBrushless);
    private CANSparkMax motorCenterstage = new CANSparkMax(ELEVATOR_MOTOR_INNER, MotorType.kBrushless);

    private SparkPIDController mainPID;
    private SparkPIDController followerPID;
    private SparkPIDController centerstagePID;

    private SparkLimitSwitch lowerLimitSwitch;
    private SparkLimitSwitch upperLimitSwitch;
  
    private RelativeEncoder mainEncoder;
    private RelativeEncoder followEncoder;
    private RelativeEncoder centerstageEncoder;

    public Elevator() {
        Util.consoleLog();

        // follower is mirrored and reversed
        // don't change this it's very important as shafts are linked with coupler
        // and will shatter if driven in opposite directions
        motorFollower.follow(motorMain, true);
        motorFollower.setInverted(true);

        motorFollower.setIdleMode(IdleMode.kBrake);
        motorMain.setIdleMode(IdleMode.kBrake);
        motorCenterstage.setIdleMode(IdleMode.kCoast);

        lowerLimitSwitch = motorFollower.getReverseLimitSwitch(Type.kNormallyOpen);
        upperLimitSwitch = motorFollower.getForwardLimitSwitch(Type.kNormallyOpen);

        lowerLimitSwitch.enableLimitSwitch(true);
        upperLimitSwitch.enableLimitSwitch(true);

        mainEncoder = motorMain.getEncoder();
        followEncoder = motorFollower.getEncoder();
        centerstageEncoder = motorCenterstage.getEncoder();

        mainPID = motorMain.getPIDController();
        configurePID(mainPID, 0.1, 0, 0);
        followerPID = motorFollower.getPIDController();
        configurePID(followerPID, 0.1, 0, 0);
        centerstagePID = motorCenterstage.getPIDController();
        configurePID(centerstagePID, 0.1, 0, 0);

        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(motorMain, DCMotor.getNEO(1));
        }
    }

    private void configurePID(SparkPIDController pidController, double p, double i, double d) {
        pidController.setP(0.1);
        pidController.setI(0);
        pidController.setD(0);
    }

    @Override
    public void periodic() {
        if (lowerLimitSwitch.isPressed()) {
            mainEncoder.setPosition(0); // reset the encoder counts
            followEncoder.setPosition(0);
            centerstageEncoder.setPosition(0);
        }
        
        AdvantageScope.getInstance().setElevatorHeight(getElevatorHeight());
        AdvantageScope.getInstance().setCarriageHeight(getCenterstageHeight());

        SmartDashboard.putNumber("Elevator1 Encoder", mainEncoder.getPosition() * ELEVATOR_WINCH_FACTOR);
        SmartDashboard.putNumber("Elevator2 Encoder", followEncoder.getPosition() * ELEVATOR_WINCH_FACTOR);
        SmartDashboard.putNumber("Centerstage Encoder", centerstageEncoder.getPosition() * ELEVATOR_CENTERSTAGE_FACTOR);

        SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
        SmartDashboard.putNumber("Centerstage Height", getCenterstageHeight());
        

    }

    /**
     * move elevator in direction based on speed
     * @param speed (such as from a joystick value)
     */
    public void move(double speed) {
        motorMain.set(-0.5 * speed);
        if (Robot.isSimulation()) setElevatorHeight(getElevatorHeight() + (speed / 100));
    }

    public void moveCenterStage(double speed) {
        motorCenterstage.set(speed);
        if (Robot.isSimulation()) setCenterstageHeight(getCenterstageHeight() + (speed / 100));
    }

    public void setElevatorHeight(double height) {
        mainPID.setReference(height / ELEVATOR_WINCH_FACTOR, ControlType.kPosition);
        // followerPID.setReference(height / ELEVATOR_WINCH_FACTOR, ControlType.kPosition);
        if (Robot.isSimulation()) mainEncoder.setPosition(height / ELEVATOR_WINCH_FACTOR);
        if (Robot.isSimulation()) followEncoder.setPosition(height / ELEVATOR_WINCH_FACTOR);
    }

    public void setCenterstageHeight(double height) {
        mainPID.setReference(height / ELEVATOR_CENTERSTAGE_FACTOR, ControlType.kPosition);
        if (Robot.isSimulation()) centerstageEncoder.setPosition(height / ELEVATOR_CENTERSTAGE_FACTOR);
    }

    public double getElevatorHeight() {
        double mainValue = mainEncoder.getPosition() * ELEVATOR_WINCH_FACTOR;
        double followValue = followEncoder.getPosition() * ELEVATOR_WINCH_FACTOR;
        return (0.5 * (mainValue + followValue)); // mean
    }

    public double getCenterstageHeight() {return centerstageEncoder.getPosition() * ELEVATOR_CENTERSTAGE_FACTOR;}

    public void resetEncoders() {
        mainEncoder.setPosition(0);
        followEncoder.setPosition(0);
        centerstageEncoder.setPosition(0);
    }

    // /**
    //  * The height of the elevator (measured at shooter pivot)
    //  * above the ground
    //  * @return the height in meters of MAXSpline shaft above ground
    //  */
    // public double getHeight() {
    //     double avgCounts = 0.5 * (mainEncoder.getPosition() + followEncoder.getPosition());
    //     return avgCounts;
    // }

}
