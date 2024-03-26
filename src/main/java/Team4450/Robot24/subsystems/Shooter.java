package Team4450.Robot24.subsystems;

import Team4450.Lib.Util;
import Team4450.Robot24.AdvantageScope;
import Team4450.Robot24.Robot;

import static Team4450.Robot24.Constants.SHOOTER_MOTOR_TOP;
import static Team4450.Robot24.Constants.SHOOTER_MOTOR_BOTTOM;
import static Team4450.Robot24.Constants.SHOOTER_MOTOR_FEEDER;
import static Team4450.Robot24.Constants.SHOOTER_MOTOR_PIVOT;

import static Team4450.Robot24.Constants.SHOOTER_SPEED;
import static Team4450.Robot24.Constants.SHOOTER_PIVOT_FACTOR;
import static Team4450.Robot24.Constants.SHOOTER_PRECISE_PIVOT_FACTOR;
import static Team4450.Robot24.Constants.SHOOTER_FEED_SPEED;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* ANGLE REFERENCE:
                   ||||
                   ||
                   ||
                 \ ||
        <-- shoot  ||------0deg-------
                   || \___
                   ||  \  \__
                   ||    \   \-39deg
                   ||      \     \__
          -90deg | ||  -50deg \    INTAKE
  =================================INTAKE  front -->
  8888                              8888
  8888                              8888
 */

/**
 * Subsystem for the Shooter subassemebly on the 2024 robot. Should not be
 * used on its own, should be contained within ElevatedShooter subsystem
 */
public class Shooter extends SubsystemBase {
    private CANSparkMax motorTop = new CANSparkMax(SHOOTER_MOTOR_TOP, MotorType.kBrushless);
    private CANSparkMax motorBottom = new CANSparkMax(SHOOTER_MOTOR_BOTTOM, MotorType.kBrushless);
    private CANSparkMax motorFeeder = new CANSparkMax(SHOOTER_MOTOR_FEEDER, MotorType.kBrushless);
    private CANSparkMax motorPivot = new CANSparkMax(SHOOTER_MOTOR_PIVOT, MotorType.kBrushless);

    public boolean hasShot = false; // used to end spin up command, should only be read publicly


    private RelativeEncoder pivotEncoder;
    private AbsoluteEncoder pivotCoolEncoder;
    private RelativeEncoder topMotorEncoder;
    private RelativeEncoder bottomMotorEncoder;

    private SparkLimitSwitch noteSensor;

    private double shooterSpeed = SHOOTER_SPEED;
    private double feedSpeed = SHOOTER_FEED_SPEED;

    private ProfiledPIDController pivotPID;
    private boolean shooterIsRunning = false, feederIsRunning = false;
    private final double PIVOT_TOLERANCE = 1; //encoder counts: note angle

    private final double PIVOT_START = -39; // angle in degrees

    private double goal = PIVOT_START;

    // NOTE: I removed the shuffleboard speed setting because they were too
    // much of a hassle to handle with all of the different speed states the shooter could be in
    // (feeding, slow feeding, inverse feeding, shooting, etc.)

    /**
     * Shooter of 2024 robot USS ProtoStar
     */
    public Shooter() {
        Util.consoleLog();

        motorBottom.follow(motorTop);
        motorFeeder.setInverted(true);

        pivotEncoder = motorPivot.getEncoder();
        pivotCoolEncoder = motorPivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        topMotorEncoder = motorTop.getEncoder();
        bottomMotorEncoder = motorBottom.getEncoder();

        resetEncoders();

        // plugges into Spark MAX itself
        noteSensor = motorFeeder.getForwardLimitSwitch(Type.kNormallyOpen);

        // profiled PID controller allows us to control acceleration and
        // deceleration and max speed of the pivot!
        pivotPID = new ProfiledPIDController(0.05, 0, 0,
            new Constraints(angleToEncoderCounts(180), angleToEncoderCounts(360)) // max velocity(/s), max accel(/s)
        );
        pivotPID.setTolerance(PIVOT_TOLERANCE); // encoder counts not degrees for this one

        Util.consoleLog("Shooter created!");
    }

    /**
     * whether the shooter posesses a note
     * @return if the robot has note (true) or note (false)
     */
    public boolean hasNote() {
        if (RobotBase.isSimulation())
            return AdvantageScope.getInstance().hasNoteSim();
        else
            return noteSensor.isPressed();
    }

    /**
     * enables the feed motor (sushi rollers) to push the Note into (or out of)
     * the rolling shooter wheels (which must be enabled seperately)
     * @param speedfactor the speed bounded [-1,1] of max feedspeed
     */
    public void startFeeding(double speedfactor) {
        SmartDashboard.putNumber("sushi", speedfactor);
        motorFeeder.set(Util.clampValue(speedfactor, 1) * feedSpeed);
        feederIsRunning = true;
        updateDS();
    }

    /*
     * set the current position as the "zero" position.
     * Careful!
     */
    public void resetEncoders() {
        pivotEncoder.setPosition(angleToEncoderCounts(-39));
        topMotorEncoder.setPosition(0);
        bottomMotorEncoder.setPosition(0);
    }

    /**
     * set the current setpoint/goal to the current position,
     * essentially "locking" the pivot in place
     */
    public void lockPosition() {
        goal = getAngle();
        pivotPID.reset(angleToEncoderCounts(goal));
    }

    /**
     * remove setpoint control, causing pivot to become limp and
     * react to external forces with no braking or anything.
     */
    public void unlockPosition() {
        goal = Double.NaN; // when setpoint NaN it doesn't do it
    }

    
    
    /**
     * set whether the note sensor triggers the feed rollers to stop or not
     * @param enabled whether the motor should pay attention to sensor or not
     */
    public void enableClosedLoopFeedStop(boolean enabled) {
        noteSensor.enableLimitSwitch(enabled);
    }

    /** stops the feed motor */
    public void stopFeeding() {
        motorFeeder.set(0);
        feederIsRunning = false;
        updateDS();
    }

    /** spins the shooter wheels */
    public void startShooting() {
        motorTop.set(shooterSpeed);
        shooterIsRunning = true;
        updateDS();
    }

    /**
     * spins the shooter wheels at the given % speed
     * @param factor bounded [-1, 1] of total max speed
     */
    public void startShooting(double factor) {
        motorTop.set(shooterSpeed * factor);
        shooterIsRunning = true;
        updateDS();
    }

    /**
     * run the shooter wheels backwards at 15% power.
     */
    public void backShoot() {
        motorTop.set(-0.15);
        shooterIsRunning = true;
        updateDS();
    }

    /** stops the shooter wheels */
    public void stopShooting() {
        motorTop.set(0);
        shooterIsRunning = false;
        updateDS();
    }

    /**
     * Sets the shooter assembly to a given angle
     * @param angle the angle in degrees
     */
    public void setAngle(double angle) {
        goal = angle;
    }

    /**
     * Get the angle of the shooter assembly
     * @return the angle in degrees
     */
    public double getAngle() {
        double roughAngle = pivotEncoder.getPosition() * SHOOTER_PIVOT_FACTOR; // convert to degrees
        if (roughAngle < 15 && roughAngle > -80 && RobotBase.isReal()) {
            return pivotCoolEncoder.getPosition() * SHOOTER_PRECISE_PIVOT_FACTOR;
        }
        return roughAngle;
    }

    /**
     * Get the average wheel speed of top and bottom
     * shooter rollers
     * @return the mean wheel speed in meters per second
     */
    public double getWheelSpeed() {
        double wheelRadius = 1.5 * 0.0254; // 1.5 in -> meters
        double topWheelSpeed = (topMotorEncoder.getVelocity() / 60.0) * wheelRadius * 2 * Math.PI; // rpm -> m/s
        double bottomWheelSpeed = (bottomMotorEncoder.getVelocity() / 60.0) * wheelRadius * 2 * Math.PI; // rpm -> m/s
        double averageWheelSpeed = 0.2 * (topWheelSpeed + bottomWheelSpeed); // mean of top and bottom
        return averageWheelSpeed;
    }

    /**
     * checks if shooter is at angle with tolerance
     * @param angle the angle to check against
     * @return true or false
     */
    public boolean isAtAngle(double angle) {
        // check if the absolute difference < tolerance
        return Math.abs(pivotEncoder.getPosition() - angleToEncoderCounts(angle)) < PIVOT_TOLERANCE;
    }

    /**
     * given an angle, return the encoder counts
     * @param angle angle of shooter position: 0 is nominal angle in degrees
     *              (see beginning of Shooter.java for reference)
     * @return the raw encoder position
     */
    private double angleToEncoderCounts(double angle) {
        return angle / SHOOTER_PIVOT_FACTOR;
    }

    /**
     * Increment or decrement the setpoint (for manual joystick use)
     * @param amount the # of degrees to change setpoint by
     */
    public void movePivotRelative(double amount) {
        goal += amount;
        // motorPivot.set(0.4*speed);
        // if (Robot.isSimulation()) pivotEncoder.setPosition(pivotEncoder.getPosition() + (0.5*speed));
    }

    @Override
    public void periodic() {
        // for simulation and logging
        AdvantageScope.getInstance().setShooterAngle(getAngle());
        SmartDashboard.putNumber("Shooter Angle", getAngle());
        SmartDashboard.putBoolean("Note Sensor", hasNote());
        SmartDashboard.putNumber("pivot_setpoint", angleToEncoderCounts(goal));
        SmartDashboard.putNumber("pivot_measured", pivotEncoder.getPosition());

        // if goal/setpoint is NaN, then just ignore the setpoint
        if (Double.isNaN(goal)) return;

        double motorOutput = pivotPID.calculate(pivotEncoder.getPosition(), angleToEncoderCounts(goal));
        motorPivot.set(motorOutput);

        // simulate shooter movement by incrementing position based on speed (not super accurate but
        // until REV adds proper Spark MAX simulation support this is what works within reason without
        // doing a ton of annoying physics I didn't want to worry about -cole)
        if (Robot.isSimulation()) pivotEncoder.setPosition(pivotEncoder.getPosition() + (2*motorOutput));
    }

    /**
     * update DriverStation status
     */
    private void updateDS()
    {
        SmartDashboard.putBoolean("Shooter", shooterIsRunning);
        SmartDashboard.putBoolean("Feeder", feederIsRunning);
    }
}