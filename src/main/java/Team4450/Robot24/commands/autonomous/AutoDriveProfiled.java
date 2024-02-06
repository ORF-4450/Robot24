package Team4450.Robot24.commands.autonomous;

import static Team4450.Robot24.Constants.*;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Robot24.RobotContainer;
import Team4450.Robot24.subsystems.DriveBase;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/**
 * A command that will drive the robot forward/backward to the specified distance using
 * a motion profiled PID command and steering correction. Velocity & acceleration are a 
 * guess, need to characterize the robot for good numbers. Motion profile will accelerate
 * the robot to an appropriate speed and decelerate to a stop at the target distance.
 */
public class AutoDriveProfiled extends ProfiledPIDCommand 
{
    private DriveBase     driveBase;

    private static double kP = 1.2, kI = .15, kD = 0, kToleranceMeters = .10;
    private double        distance, startTime;
    private int           iterations;
    private StopMotors    stop;
    private Brakes        brakes;
    
    /**
     * Drives robot to the specified distance using a motion profile with straight steering.
     *
     * @param drive         The drive subsystem to use.
     * @param distance      The distance to drive in meters. + is forward.
     * @param stop          Set to stop or not stop motors at end.
     * @param brakes        If stopping, set brakes on or off.
     */
    public AutoDriveProfiled(DriveBase driveBase, double distance, StopMotors stop, Brakes brakes) 
    {
        super(
            new ProfiledPIDController(kP, kI, kD, 
                                      new TrapezoidProfile.Constraints(AutoConstants.kMaxSpeedMetersPerSecond, 
                                      AutoConstants.kMaxAccelerationMetersPerSecondSquared)),
            // Closed loop on distance via reference so pid controller can call it on each execute() call.
            driveBase::getDistanceTraveled,
            // Set target distance.
            distance,
            // Pipe output to drive robot.
            (output, setpoint) -> driveBase.drive(output, 0, 0, false)); //,
            // Require the drive base. Note: the db is required by the calling
            // autonmous command so we don't need it here and doing it here will
            // interrupt the calling auto command.
            //);

        Util.consoleLog("distance=%.3fm  stop=%s  brakes=%s", distance, stop, brakes);
            
        Util.consoleLog("kP=%.6f  kI=%.6f", kP, kI);

        this.driveBase = driveBase;
        this.brakes = brakes;
        this.stop = stop;
        this.distance = distance;

        getController().setTolerance(kToleranceMeters);
    }
        
    @Override
    public void initialize()
    {
        Util.consoleLog("distance=%.3fm  stop=%s  brakes=%s", distance, stop, brakes);

        startTime = Util.timeStamp();

        if (brakes == Brakes.on)
            driveBase.setBrakeMode(true);
        else
            driveBase.setBrakeMode(false);
        
        driveBase.resetDistanceTraveled();
        
        driveBase.resetYaw();
    }
    
    public void execute() 
    {
        Util.consoleLog();
        
        double yaw = -driveBase.getYaw();   // Invert to swerve angle convention.

        super.execute();

        LCD.printLine(LCD_5, "Wheel distance=%.3f", driveBase.getDistanceTraveled());

        Util.consoleLog("tg=%.3f  dist=%.3f  sp=%.3f err=%.3f  yaw=%.2f", 
                        getController().getGoal().position, driveBase.getDistanceTraveled(), 
                        getController().getSetpoint().position, getController().getPositionError(), yaw);

        iterations++;
    }

    @Override
    public boolean isFinished() 
    {
        // End when the target distance is reached. The PIDController atGoal and atSetPoint
        // functions do not seem to work. Don't know why.

        return (Math.abs(distance - driveBase.getDistanceTraveled())) <= kToleranceMeters;
    }
	
	@Override
	public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
        
		if (stop == StopMotors.stop) driveBase.stop();
		
		double actualDist = Math.abs(driveBase.getDistanceTraveled());
		
        Util.consoleLog("end: target=%.3f  actual=%.3f  error=%.2f pct  yaw=%.2f hdng=%.2f", 
                Math.abs(distance), actualDist, (actualDist - Math.abs(distance)) / Math.abs(distance) * 100.0, 
                driveBase.getYaw(), RobotContainer.navx.getHeading());
        
		Util.consoleLog("iterations=%d  elapsed time=%.3fs", iterations, Util.getElaspedTime(startTime));
        
		Util.consoleLog("end ---------------------------------------------------------------");
	}

    public enum Brakes
	{
		off,
		on
	}
	
	public enum Pid
	{
		off,
		on
	}
	
	public enum Heading
	{
		angle,
        heading,
        totalAngle
	}
	
	public enum StopMotors
	{
		dontStop,
		stop
	}	
}