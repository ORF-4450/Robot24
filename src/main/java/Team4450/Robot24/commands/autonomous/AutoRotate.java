package Team4450.Robot24.commands.autonomous;

import Team4450.Lib.Util;
import Team4450.Robot24.RobotContainer;
import Team4450.Robot24.subsystems.DriveBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * A command that will turn the robot to the specified angle using a PID command.
 * Acceleration is a guess, need to characterize the robot for good numbers.
 */
public class AutoRotate extends PIDCommand 
{
    private DriveBase    driveBase;
    
    private static AutoRotate   thisInstance;

    private static double kP = .003, kI = kP, kD = 0, kToleranceDeg = .5 ;
    private double        startTime;
    private int           iterations;

    /**
     * Turns to robot the specified angle using a PID controller.
     *
     * @param drive       The drive subsystem to use.
     * @param targetAngle The angle to turn to + is counter clockwise (left).
     */
    public AutoRotate(DriveBase driveBase, double targetAngle) 
    {
        // Since we are extending PIDCommand, we will call the underlying constructor
        // to instantiate the components of PIDCommand.
        super(
            new PIDController(kP, kI, kD),
            // Closed loop on yaw via reference so pid controller can call it on each execute() call.
            // The measurement.
            driveBase::getYaw,
            // Set target angle (setpoint)
            targetAngle,
            // Pipe output to turn robot
            output -> thisInstance.drive(0, 0, output)); //,
            // Require the drive base
            //drive);

        Util.consoleLog("angle=%.2f  kP=%.3f  kI=%.3f", targetAngle, kP, kI);

        this.driveBase = driveBase;
        thisInstance = this;

        // Set the controller tolerance.

        getController().setTolerance(kToleranceDeg);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();
        
        startTime = Util.timeStamp();

        // Try to prevent over rotation.
        //driveBase.SetCANTalonBrakeMode(true);
        
        // Reset gyro yaw to zero
        driveBase.resetYaw();
    }

    @Override
    public void execute()
    {
        // Run the underlying PIDCommand which calls uses PIDController to compute
        // the motor output and send it to the consumer we defined above (driveBase.drive).
        super.execute();

        Util.consoleLog("sp=%.3f  m=%.3f  err=%.3f", getController().getSetpoint(), 
                        m_measurement.getAsDouble(), getController().getPositionError());

        Util.consoleLog("yaw=%.2f  hdng=%.2f", driveBase.getYaw(), RobotContainer.navx.getHeading());

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
        // End when the controller is at the reference.
        //return getController().atSetpoint();
        // We use this manual check of reaching the setpoint because the atSetPoint()
        // function did work..?..
        return Math.abs(getController().getPositionError()) <= kToleranceDeg;
    }
  	
	@Override
	public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
		
		driveBase.stop();

		Util.consoleLog("hdg=%.2f  target=%.2f  yaw=%.2f  error=%.2f pct", RobotContainer.navx.getHeading(), 
                        getController().getSetpoint(), driveBase.getYaw(),
                        (driveBase.getYaw() - Math.abs(getController().getSetpoint())) / 
                        Math.abs(getController().getSetpoint()) * 100.0);
        
		Util.consoleLog("iterations=%d  elapsed time=%.3fs", iterations, Util.getElaspedTime(startTime));

        Util.consoleLog("end ----------------------------------------------------------");
	}
}