package Team4450.Robot24.commands.autonomous;

import static Team4450.Robot24.Constants.*;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Robot24.RobotContainer;
import Team4450.Robot24.commands.autonomous.AutoDriveProfiled.Brakes;
import Team4450.Robot24.commands.autonomous.AutoDriveProfiled.Pid;
import Team4450.Robot24.commands.autonomous.AutoDriveProfiled.StopMotors;
import Team4450.Robot24.subsystems.DriveBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This is an example autonomous command using 4450 written library of auto driving
 * support commands.
 */
public class TestAuto1 extends Command
{
	private final DriveBase driveBase;
	
	private SequentialCommandGroup	commands = null;
	private Command					command = null;
	private	Pose2d					startingPose;

	/**
	 * Creates a new TestAuto1 autonomous command. This command demonstrates one
	 * possible structure for an autonomous command and shows the use of the 
	 * autonomous driving support commands.
	 *
	 * @param driveBase DriveBase subsystem used by this command to drive the robot.
	 */
	public TestAuto1(DriveBase driveBase, Pose2d startingPose) 
	{
		Util.consoleLog();
		
		this.driveBase = driveBase;

		this.startingPose = startingPose;
			  
		// Use addRequirements() here to declare subsystem dependencies.
		// This command is requiring the driveBase for itself and all 
		// commands added to the command list. If any command in the
		// list also requires the drive base it will cause this command
		// to be interrupted.
		addRequirements(this.driveBase);
	}
	
	/**
	 * Called when the command is initially scheduled. (non-Javadoc)
	 * @see edu.wpi.first.wpilibj2.command.Command#initialize()
	 */
	@Override
	public void initialize() 
	{
		Util.consoleLog();
		
		//driveBase.setMotorSafety(false);  // Turn off watchdog.
		
	  	LCD.printLine(LCD_1, "Mode: Auto - TestAuto1 - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
				DriverStation.isFMSAttached(), gameMessage);

		SmartDashboard.putBoolean("Autonomous Active", true);

		// Set heading tracking to initial angle (0 is robot pointed down the field) so
		// NavX class can track which way the robot is pointed all during the match.
		RobotContainer.navx.setHeading(startingPose.getRotation().getDegrees());
			
		// Target heading should be the same.
		RobotContainer.navx.setTargetHeading(startingPose.getRotation().getDegrees());
			
		// Reset odometry tracking with initial x,y position and heading (set above) specific to this 
		// auto routine. Robot must be placed in same starting location each time for pose tracking
		// to work.
		driveBase.resetOdometry(startingPose);
		
		// Since a typical autonomous program consists of multiple actions, which are commands
		// in this style of programming, we will create a list of commands for the actions to
		// be taken in this auto program and add them to a sequential command list to be 
		// executed one after the other until done.
		
		commands = new SequentialCommandGroup();
		
		// First action is to drive forkward 1 meters and stop.
		
		command = new AutoDriveProfiled(driveBase, 1, StopMotors.stop, Brakes.on);
		
		//commands.addCommands(command);
		
		// Next action is to drive backward 1 meters and stop. 
		
		command = new AutoDriveProfiled(driveBase, -1, StopMotors.stop, Brakes.on);
		
		//commands.addCommands(command);

		// Now strafe left 1 meters.

		command = new AutoStrafeProfiled(driveBase, 1, StopMotors.stop, Brakes.on);
		
		//commands.addCommands(command);

		// Now strafe right 1 meters.

		command = new AutoStrafeProfiled(driveBase, -1, StopMotors.stop, Brakes.on);
		
		//commands.addCommands(command);

		// Now rotate 90 degrees left.

		command = new AutoRotate(driveBase, 90);

		//commands.addCommands(command);

		// Now rotate 180 degrees right.

		command = new AutoRotateProfiled(driveBase, -180);

		//commands.addCommands(command);

		// Now rotate 90 degrees left.

		command = new AutoRotateProfiled(driveBase, 90);

		//commands.addCommands(command);

		// Now drive ahead 1.683 meter to charge station.

		//ommand = new AutoDrive(driveBase, .50, 1.683, StopMotors.stop, Brakes.on, Pid.on);

		command = new AutoDrive(driveBase, .50, 3.429, StopMotors.stop, Brakes.on, Pid.on);

		commands.addCommands(command);

		// Launch autonomous command sequence.
		
		commands.schedule();
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
		Util.consoleLog("interrupted=%b", interrupted);
		
		driveBase.drive(0, 0, 0, false);
		
		Util.consoleLog("final heading=%.2f  Radians=%.2f", RobotContainer.navx.getHeading(), RobotContainer.navx.getHeadingR());
		Util.consoleLog("end ---------------------------------------------------------------");

		SmartDashboard.putBoolean("Autonomous Active", false);
	}
	
	/**
	 *  Returns true when this command should end. That should be when
	 *  all the commands in the command list have finished.
	 */
	@Override
	public boolean isFinished() 
	{
		// Note: commands.isFinished() will not work to detect the end of the command list
		// due to how FIRST coded the SquentialCommandGroup class. 
		
		return !commands.isScheduled();
	}
}

