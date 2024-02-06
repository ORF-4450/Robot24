package Team4450.Robot24.commands.autonomous;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Robot24.RobotContainer;
import Team4450.Robot24.commands.autonomous.AutoDriveProfiled.Brakes;
import Team4450.Robot24.commands.autonomous.AutoDriveProfiled.StopMotors;
import Team4450.Robot24.subsystems.DriveBase;

import static Team4450.Robot24.Constants.*;

import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This is an example autonomous command based on 4450 customized version of
 * Wpilib Trajectory following commands.
 */
public class TestAuto3 extends Command
{
	private final DriveBase driveBase;
	
	private SequentialCommandGroup	commands = null;
    private Command					command = null;
	private	Pose2d					startingPose;   

	/**
	 * Creates a new TestAuto3 autonomous command. This command demonstrates one
	 * possible structure for an autonomous command and shows the use of the 
	 * autonomous driving support commands.
	 *
	 * @param driveBase DriveBase subsystem used by this command to drive the robot.
	 * @param startingPose Starting pose of the robot.
	 */
	public TestAuto3(DriveBase driveBase, Pose2d startingPose) 
	{
		Util.consoleLog();
		
		this.driveBase = driveBase;

		this.startingPose = startingPose;

		// Use addRequirements() here to declare subsystem dependencies.
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

		SmartDashboard.putBoolean("Autonomous Active", true);
		
	  	LCD.printLine(LCD_1, "Mode: Auto - TestAuto3 - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
				DriverStation.isFMSAttached(), gameMessage);
		
		// Reset drive base distance tracking.	  	
	  	driveBase.resetDistanceTraveled();
	  	
	  	// Set drive base yaw tracking to 0.
	  	driveBase.resetYaw();

		// Set heading to initial angle (0 is robot pointed down the field) so
		// NavX class can track which way the robot is pointed all during the match.
		RobotContainer.navx.setHeading(startingPose.getRotation().getDegrees());
			
		// Target heading should be the same.
		RobotContainer.navx.setTargetHeading(startingPose.getRotation().getDegrees());
		
		// Since a typical autonomous program consists of multiple actions, which are commands
		// in this style of programming, we will create a list of commands for the actions to
		// be taken in this auto program and add them to a sequential command list to be 
		// executed one after the other until done.
		
		commands = new SequentialCommandGroup();
		
		// We will create a trajectory and set the robot to follow it.
    
        TrajectoryConfig config = AutoDriveTrajectory.getTrajectoryConfig();

		// NOTE: Here we run into the differencec between X/Y axis definitions between
		// joystick axes and field axes. JS is Y fwd/back down the field and X is left/right
		// paralell to the driver station wall. In the WPILib code, it is opposite. Note that
		// in the drive command we switch the JS axes to fix this issue before passing to the
		// swerve code. Here we are dealing directly with swerve drive code so X is fwd/back
		// down the field and Y is left/right (strafe).

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                        // Start at the origin set above
                                        startingPose,
                                        // Pass through these two interior waypoints, making an 's' curve path
                                        List.of(
                                            new Translation2d(startingPose.getX() + 1, startingPose.getY() + 1),
                                            new Translation2d(startingPose.getX() + 1, startingPose.getY() - 1)
                                        ),
                                        // End 3 meters straight ahead of where we started, facing forward
                                        new Pose2d(startingPose.getX() + 3, startingPose.getY(), startingPose.getRotation()),
                                        // Pass config
                                        config);		

        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //                                 // Start at the origin set above
        //                                 startingPose,
        //                                 // Pass through these interior waypoints
        //                                 List.of(
        //                                     new Translation2d(startingPose.getX() + 1, startingPose.getY()),
        //                                     new Translation2d(startingPose.getX() + 1.5, startingPose.getY() + .5),
        //                                     new Translation2d(startingPose.getX() + 2.0, startingPose.getY() + 1.0),
        //                                     new Translation2d(startingPose.getX() + 1.5, startingPose.getY() + 1.5),
        //                                     new Translation2d(startingPose.getX() + 1, startingPose.getY() + 2.0)
        //                                     ),
        //                                 // End back where we started but left 2m, facing 180 from start.
        //                                 new Pose2d(startingPose.getX(), startingPose.getY() + 2.0, new Rotation2d(Math.toRadians(180))),
        //                                 // Pass config
        //                                 config);		
        

        command = new AutoDriveTrajectory(driveBase, exampleTrajectory, StopMotors.stop, Brakes.on);
        //command = new AutoDriveTrajectory(driveBase, RobotContainer.testTrajectory, StopMotors.stop, Brakes.on);
		//command = new AutoDriveTrajectory(driveBase, RobotContainer.ppTestTrajectory, StopMotors.stop, Brakes.on);
		
		commands.addCommands(command);
		
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
		
		driveBase.stop();
		
		Util.consoleLog("final heading=%.2f  Radians=%.2f", RobotContainer.navx.getHeading(), RobotContainer.navx.getHeadingR());
		Util.consoleLog("end -----------------------------------------------------");

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
