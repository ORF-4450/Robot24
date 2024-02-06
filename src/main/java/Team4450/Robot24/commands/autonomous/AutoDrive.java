package Team4450.Robot24.commands.autonomous;

import static Team4450.Robot24.Constants.*;

import Team4450.Lib.LCD;
import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot24.commands.autonomous.AutoDriveProfiled.Brakes;
import Team4450.Robot24.commands.autonomous.AutoDriveProfiled.Pid;
import Team4450.Robot24.commands.autonomous.AutoDriveProfiled.StopMotors;
import Team4450.Robot24.subsystems.DriveBase;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoDrive extends Command
{
	private final DriveBase driveBase;

	private double			elapsedTime = 0, targetDistance;
	private double			kP = 1.0, kI = kP / 10, kD = 0;
	private double			speed, startTime; 
    private int 			iterations; 
	private StopMotors 		stop;
	private Brakes 			brakes;
	private Pid 			pid;
	
	SynchronousPID			pidController = null;

	/**
	 * Creates a new AutoDrive command.
	 * 
	 * Auto drive straight in current direction and power for specified distance. Stops
	 * with or without brakes on CAN bus drive system.
	 *
	 * @param driveBase The DriveBase subsystem used by this command to drive the robot.
	 * @param speed % of max robot speed applied, + is forward.
	 * @param targetDistance Target distance to move in meters, always +.
	 * @param stop Stop stops motors at end of move, dontStop leaves power on to flow into next move.
	 * @param brakes Brakes on or off.
	 * @param pid On is use PID to control movement, off is simple drive.
	*/
	public AutoDrive(DriveBase driveBase, 
					 double speed, 
					 double targetDistance, 
					 StopMotors stop, 
					 Brakes brakes, 
					 Pid pid) 
	{
		this.driveBase = driveBase;

		Util.consoleLog("spd=%.2f  tgt=%.2f  stop=%s  brakes=%s  pid=%s", speed, targetDistance, stop, brakes, 
						pid);

		Util.checkRange(speed, 1.0);
		
		if (targetDistance < 0) throw new IllegalArgumentException("Distance < 0");
			  
		this.speed = speed;
		this.targetDistance = targetDistance;
		this.stop = stop;
		this.brakes = brakes;
		this.pid = pid;
		
		Util.consoleLog("kP=%.6f  kI=%.6f", kP, kI);
	}

	@Override
	public void initialize()
	{
        Util.consoleLog("spd=%.2f  tgt=%.2f  stop=%s  brakes=%s  pid=%s", speed, targetDistance, stop, 
                        brakes, pid);

		startTime = Util.timeStamp();
		
		if (brakes == Brakes.on)
			driveBase.setBrakeMode(true);
		else
			driveBase.setBrakeMode(false);
			
		driveBase.resetDistanceTraveled();		
		
		// If using PID to control distance, configure the PID object.
		
		if (pid == Pid.on)
		{
			pidController = new SynchronousPID("AutoDrive", kP, kI, kD);
			
			if (speed < 0)
			{
				pidController.setSetpoint(-targetDistance);
				pidController.setOutputRange(speed, 0);
			}
			else
			{
				pidController.setSetpoint(targetDistance);
				pidController.setOutputRange(0, speed);
			}

			// Start elapsed time tracking.
			
			Util.getElaspedTime();
		}
	}
	
	@Override
	public void execute() 
	{
        double actualDistance = driveBase.getDistanceTraveled();

		Util.consoleLog();

		LCD.printLine(LCD_5, "distance traveled=%.2f", actualDistance);

		// Use PID to determine the power applied. Should reduce power as we get close
		// to the target encoder value.
		
		if (pid == Pid.on)
		{
			elapsedTime = Util.getElaspedTime();
			
			speed = pidController.calculate(actualDistance, elapsedTime);
			
			Util.consoleLog("actdist=%.2f  error=%.2f  speed=%.3f  time=%.3f", actualDistance, 
							pidController.getError(), speed, elapsedTime);
		}
		else
			Util.consoleLog("tgt=%.2f  act=%.2f", targetDistance, Math.abs(actualDistance));
		
		driveBase.drive(speed, 0, 0, false);

		iterations++;
	}
	
	@Override
	public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
		
		if (stop == StopMotors.stop) driveBase.stop();
		
		double actualDistance = Math.abs(driveBase.getDistanceTraveled());
		
		Util.consoleLog("target dist=%.2f  actual dist=%.2f  error=%.2f pct", targetDistance, actualDistance, 
				(actualDistance - targetDistance) / targetDistance * 100.0);
        
		Util.consoleLog("iterations=%d  elapsed time=%.3fs", iterations, Util.getElaspedTime(startTime));

		Util.consoleLog("end -----------------------------------------------------");
	}
	
	@Override
	public boolean isFinished() 
	{
		return Math.abs(driveBase.getDistanceTraveled()) >= targetDistance;	
	}	
}
