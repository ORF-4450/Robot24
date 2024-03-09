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
import static Team4450.Robot24.Constants.SHOOTER_FEED_SPEED;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for the Shooter subassemebly on the 2024 robot
 */
public class Shooter extends SubsystemBase {
    private CANSparkMax motorTop = new CANSparkMax(SHOOTER_MOTOR_TOP, MotorType.kBrushless);
    private CANSparkMax motorBottom = new CANSparkMax(SHOOTER_MOTOR_BOTTOM, MotorType.kBrushless);
    private CANSparkMax motorFeeder = new CANSparkMax(SHOOTER_MOTOR_FEEDER, MotorType.kBrushless);
    private CANSparkMax motorPivot = new CANSparkMax(SHOOTER_MOTOR_PIVOT, MotorType.kBrushless);

    public boolean hasShot = false;


    private RelativeEncoder pivotEncoder;
    private RelativeEncoder topMotorEncoder;
    private RelativeEncoder bottomMotorEncoder;

    private SparkLimitSwitch noteSensor;

    private double shooterSpeed = SHOOTER_SPEED;
    private double feedSpeed = SHOOTER_FEED_SPEED;

    private PIDController pivotPID;
    private boolean shooterIsRunning = false, feederIsRunning = false;
    private final double PIVOT_TOLERANCE = 1; //counts

    private final double PIVOT_START = -39;

    private double setpoint = PIVOT_START;

    // NOTE: I removed the shuffleboard speed setting because they were too
    // much of a hassle to handle with all of the different speed states the shooter could be in
    // (feeding, slow feeding, inverse feeding, shooting, etc.)

    public Shooter() {
        Util.consoleLog();

        motorBottom.follow(motorTop);
        motorFeeder.setInverted(true);

        pivotEncoder = motorPivot.getEncoder();
        topMotorEncoder = motorTop.getEncoder();
        bottomMotorEncoder = motorBottom.getEncoder();

        resetEncoders();

        noteSensor = motorFeeder.getForwardLimitSwitch(Type.kNormallyOpen);

        pivotPID = new PIDController(0.05, 0, 0);
        pivotPID.setTolerance(PIVOT_TOLERANCE);

        Util.consoleLog("Shooter created!");
    }

    public boolean hasNote() {
        return noteSensor.isPressed();
    }

    /**
     * enables the feed motor (sushi rollers) to push the Note into
     * the rolling shooter wheels (which must be enabled seperately)
     */
    public void startFeeding(double speedfactor) {
        SmartDashboard.putNumber("sushi", speedfactor);
        motorFeeder.set(Util.clampValue(speedfactor, 1) * feedSpeed);
        feederIsRunning = true;
        updateDS();
    }

    public void resetEncoders() {
        pivotEncoder.setPosition(angleToEncoderCounts(-39));
        topMotorEncoder.setPosition(0);
        bottomMotorEncoder.setPosition(0);
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

    /** spins the shooter wheels in preperation for a Note */
    public void startShooting() {
        motorTop.set(shooterSpeed);
        shooterIsRunning = true;
        updateDS();
    }

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
        setpoint = angle;
    }

    /**
     * Get the angle of the shooter assembly
     * @return the angle in degrees
     */
    public double getAngle() {
        return pivotEncoder.getPosition() * SHOOTER_PIVOT_FACTOR;
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
     * checks if shooter is at angle with 3 deg tolerance
     * @param angle the angle to check against
     * @return
     */
    public boolean isAtAngle(double angle) {
        // if (Double.isNaN(setpoint))
        //     return true;
        return Math.abs(pivotEncoder.getPosition() - angleToEncoderCounts(angle)) < PIVOT_TOLERANCE;
    }

    /**
     * given an angle, return the encoder counts
     * @param angle angle of shooter position: 0 is nominal angle in degrees
     * @return the raw encoder position
     */
    private double angleToEncoderCounts(double angle) {
        return angle / SHOOTER_PIVOT_FACTOR;
    }

    /**
     * Sets the shooter assembly speed (for manual joystick use)
     * @param speed the speed
     */
    public void movePivotRelative(double speed) {
        setpoint += speed;
        // motorPivot.set(0.4*speed);
        // if (Robot.isSimulation()) pivotEncoder.setPosition(pivotEncoder.getPosition() + (0.5*speed));
    }

    @Override
    public void periodic() {
        AdvantageScope.getInstance().setShooterAngle(getAngle());
        SmartDashboard.putNumber("Shooter Angle", getAngle());
        // SmartDashboard.putNumber("pivot_measured", getAngle());
        SmartDashboard.putBoolean("Note Sensor", hasNote());

        SmartDashboard.putNumber("pivot_setpoint", angleToEncoderCounts(setpoint));
        double motorOutput = pivotPID.calculate(pivotEncoder.getPosition(), angleToEncoderCounts(setpoint));
        SmartDashboard.putNumber("pivot_measured", pivotEncoder.getPosition());
        motorPivot.set(motorOutput);
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