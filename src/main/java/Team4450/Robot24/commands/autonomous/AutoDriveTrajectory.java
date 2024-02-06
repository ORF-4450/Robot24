package Team4450.Robot24.commands.autonomous;

import static Team4450.Robot24.Constants.*;

import Team4450.Lib.Util;
import Team4450.Robot24.commands.autonomous.AutoDriveProfiled.Brakes;
import Team4450.Robot24.commands.autonomous.AutoDriveProfiled.StopMotors;
import Team4450.Robot24.subsystems.DriveBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/**
 * A command that will follow a trajectory using a SwerveDriveController. All of the parameters
 * below are a guess. Need to characterize the robot to get good numbers. Works so-so with
 * the guesses.
 * NOTE: There are WPILib limitations in path following with swerve. See the warnings in the
 * Path Planing section of the WPILib doc.
 */
public class AutoDriveTrajectory extends SwerveControllerCommand
{
    private static double   kPX = 1.0, kIX = kPX / 100, kDX = 0, startTime;
    private static double   kPY = 1.0, kIY = kPY / 100, kDY = 0;
    private static double   kPTheta = 1.0, KiTheta = kPTheta / 100, KDTheta = 0;
    private int             iterations;

    private DriveBase       driveBase;
    private Trajectory      trajectory;
    private StopMotors      stop;
    private Brakes          brakes;
    
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    
    private static ProfiledPIDController thetaController = 
                new ProfiledPIDController(kPTheta, KiTheta, KDTheta, kThetaControllerConstraints);

    /**
     * Auto drive the given trajectory.
     * @param driveBase     Drive base to use.
     * @param trajectory    Trajectory to follow.
     * @param stop          Set stop or not at trajectory end.
     * @param brakes        If stopping, set if brakes on or off.
     */
    AutoDriveTrajectory(DriveBase driveBase, Trajectory trajectory, StopMotors stop, Brakes brakes)
    {
        super(trajectory,
            driveBase::getPose, 
            DriveConstants.kDriveKinematics,
            new PIDController(kPX, 0, 0),
            new PIDController(kPY, 0, 0),
            thetaController,
            driveBase::setModuleStates);

        this.driveBase = driveBase;
        this.trajectory = trajectory;
        this.stop = stop;
        this.brakes = brakes;
    }
    
    @Override
    public void initialize()
    {
        Util.consoleLog();
    
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        startTime = Util.timeStamp();
        
        super.initialize();

        if (brakes == Brakes.on)
            driveBase.setBrakeMode(true);
        else
            driveBase.setBrakeMode(false);

        // Set the current robot pose to match the starting pose of the trajectory. If all of your
        // autonomouse moves are correctly coordinated the starting pose of the trajectory should
        // match the physical pose of the robot.
        
        Pose2d pose = trajectory.getInitialPose();

        Util.consoleLog("initial traj poseX=%.2f  poseY=%.2f  poseHdg=%.2f", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
        
        //driveBase.setOdometry(pose);
    }

    @Override
    public void execute()
    {
        Util.consoleLog();

        // Causes the controller to advance one step, driving the robot one step along the path.
        super.execute();
        
        Pose2d pose = driveBase.getPose();

        Util.consoleLog("robot poseX=%.2f  poseY=%.2f  poseAng=%.2f", pose.getX(), pose.getY(), pose.getRotation().getDegrees());

        iterations++;
    }
    
    @Override
    public boolean isFinished() 
    {
        // End when the controller has finished the trajectory.
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
        
        super.end(interrupted);

        if (stop == StopMotors.stop) driveBase.stop();
                
        Pose2d pose = driveBase.getPose();

        Util.consoleLog("poseX=%.2f  poseY=%.2f  poseAng=%.2f", pose.getX(), pose.getY(), pose.getRotation().getDegrees());

		Util.consoleLog("iterations=%d  elapsed time=%.3fs", iterations, Util.getElaspedTime(startTime));

        Util.consoleLog("end ----------------------------------------------------------");
    }

    /**
     * Generate a TrajectoryConfig given wheel speed info.
     * @return The TrajectoryConfig.
     */
    public static TrajectoryConfig getTrajectoryConfig()
    {
        return new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
                                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(DriveConstants.kDriveKinematics);
    }
}
