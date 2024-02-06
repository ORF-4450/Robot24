package Team4450.Robot24.commands.autonomous;

import static Team4450.Robot24.Constants.*;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Robot24.Constants;
import Team4450.Robot24.RobotContainer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This an autonomous command used to log the start of a PathPlanner
 * "Auto". Called from all PP Autos. Also resets tracking functions
 * in DriveBase.
 */
public class AutoStart extends Command
{

	/**
	 * Creates a new AutoStart autonomous command. 
	 */
	public AutoStart() 
	{
		Util.consoleLog();
	}
	
	/**
	 * Called when the command is initially scheduled. 
	 * @see edu.wpi.first.wpilibj2.command.Command#initialize()
	 */
	@Override
	public void initialize() 
	{
		Util.consoleLog();
		
	  	LCD.printLine(LCD_1, "Mode: Auto - %s - All=%s, Location=%d, FMS=%b, msg=%s", 
				RobotContainer.getAutonomousCommandName(), alliance.name(), location, 
				DriverStation.isFMSAttached(), gameMessage);

        RobotContainer.driveBase.resetDistanceTraveled();
        RobotContainer.driveBase.resetYaw();
        RobotContainer.driveBase.zeroGyro();

		SmartDashboard.putBoolean("Autonomous Active", true);
	}
	
	/**
	 *  Called every time the scheduler runs while the command is scheduled.
	 *  In this model, this command just idles while the Command Group we
	 *  created runs on its own executing the steps (commands) of this Auto
	 *  program.
	 */
	@Override
	public void execute() 
	{
	}
	
	/**
	 *  Called when the command ends or is interrupted.
	 */
	@Override
	public void end(boolean interrupted) 
	{
		//Util.consoleLog("interrupted=%b", interrupted);
	}
	
	/**
	 *  Returns true when this command should end. This command only runs one time.
	 */
	@Override
	public boolean isFinished() 
	{
		return true;
	}
}


