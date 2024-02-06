package Team4450.Robot24.commands.autonomous;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This an autonomous command used to log the end of a PathPlanner
 * "Auto". Called from all PP Autos.
 */
public class AutoEnd extends Command
{

	/**
	 * Creates a new AutoEnd autonomous command. 
	 */
	public AutoEnd() 
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
				
	  	//LCD.printLine(LCD_1, "Mode: Auto - DriveOut - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
		//		DriverStation.isFMSAttached(), gameMessage);

		SmartDashboard.putBoolean("Autonomous Active", false);
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


