package Team4450.Robot24.commands.autonomous;

import static Team4450.Robot24.Constants.*;

import Team4450.Lib.Util;
import Team4450.Robot24.RobotContainer;
import Team4450.Robot24.subsystems.DriveBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/**
 * A command that will turn the robot to the specified angle using a motion profiled
 * PID command. Velocity and accelleration are a guess, need to characterize the robot 
 * for good numbers.
 */
public class AutoRotateProfiled extends ProfiledPIDCommand 
{
    private DriveBase    driveBase;

    private static AutoRotateProfiled   thisInstance;

    // Note: kP was 15 but sim does not work right as 15 causes to much rotation. 5 works in sim.
    // But, 5 may be too little for a real robot and not output enough power to move. Need to test.
    private static double kP = 1.7, kI = kP / 10, kD = .02, kToleranceRad = .01;
    private double        startTime, targetAngle;
    private int           iterations;

    // We work in degrees but the profile works in radians, so we convert. 70 d/s is an eyeball
    // estimate of rotational vel and acceleration is a guess.
    private static double kMaxRotationVelrs = Math.toRadians(AutoConstants.kMaxAngularSpeedRadiansPerSecond);       
    private static double kMaxRotationAccelrss = Math.toRadians(AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);   

    /**
     * Turns to robot to the specified angle using a motion profile.
     *
     * @param driveBase   The drive subsystem to use.
     * @param targetAngle The angle to turn to + is left (counterclockwise).
     */
    public AutoRotateProfiled(DriveBase driveBase, double targetAngle) 
    {
        // Since we are extending ProfiledPIDCommand, we will call the underlying constructor
        // to instantiate the components of ProfiledPIDCommand. For reasons I cannot explain,
        // the PID function won't work with degrees but will work with radians.
        super(
            new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxRotationVelrs, 
                                                                                   kMaxRotationAccelrss)),
            // Closed loop on yaw via reference so pid controller can call it on each execute() call.
            driveBase::getYawR,
            // Set target angle
            Math.toRadians(targetAngle),
            // Pipe output to turn robot
           (output, setpoint) -> thisInstance.drive(0, 0, output));

        Util.consoleLog("angle=%.2f  kP=%.3f  kI=%.3f", targetAngle, kP, kI);

        this.driveBase = driveBase;
        thisInstance = this;
        this.targetAngle = targetAngle;

        // Set the controller tolerance.
        getController().setTolerance(kToleranceRad);
    }
    
    @Override
    public void initialize()
    {
        Util.consoleLog();
        
        startTime = Util.timeStamp();

        // Try to prevent over rotation.
        //driveBase.SetCANTalonBrakeMode(true);
        
        // Reset gyro yaw to zero.
        driveBase.resetYaw();
    }

    @Override
    public void execute()
    {
        // Run the underlying ProfiledPIDCommand which calls ProfiledPIDController to compute
        // the motor outputs and send them to the consumer we defined above (drive).
        super.execute();

        Util.consoleLog("goal=%.3fr  sp=%.3fr  m=%.3fr  err=%.3f", getController().getGoal().position,
                        getController().getSetpoint().position, m_measurement.getAsDouble(),
                        getController().getPositionError());

        Util.consoleLog("yaw=%.2f  hdng=%.2f ", driveBase.getYaw(), RobotContainer.navx.getHeading());

        iterations++;
    }

    // This method, instead of using driveBase.drive in the PID output statement
    // allows us to capture the output of the PID command and log it and apply
    // the sign change before sending on to driveBase.drive.
    private void drive(double throttle, double strafe, double rotation)
    {
        rotation = Util.clampValue(rotation, 1.0);
        
        Util.consoleLog("t=%.4f  s=%.4f  rot=%.5f", throttle, strafe, rotation);
        
        // Have to invert for sim...not sure why.
        if (RobotBase.isSimulation()) rotation *= -1;

        driveBase.drive(throttle, strafe, rotation, false);
    }

    @Override
    public boolean isFinished() 
    {
        // End when the controller is at the setpoint.
        //return getController().atGoal();
        
        return Math.abs((getController().getGoal().position - m_measurement.getAsDouble())) <= kToleranceRad;
    }
  	
	@Override
	public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
		
		driveBase.stop();
		
        double actualYaw = driveBase.getYaw();

		Util.consoleLog("hdg=%.2f  target=%.2f  yaw=%.2f  error=%.2f pct", RobotContainer.navx.getHeading(), 
                        targetAngle, actualYaw,
                        (targetAngle - actualYaw) / Math.abs(targetAngle) * 100.0);
        
		Util.consoleLog("iterations=%d  elapsed time=%.3fs", iterations, Util.getElaspedTime(startTime));

        Util.consoleLog("end ----------------------------------------------------------");
	}
}