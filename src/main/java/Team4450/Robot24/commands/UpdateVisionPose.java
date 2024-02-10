package Team4450.Robot24.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import Team4450.Lib.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot24.Robot;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.PhotonVision;

/**
 * This class runs as the default command of a PhotonVision
 * subsystem and regularly updates the SwerveDrivePoseEstimator
 * object with timestamped vision poses. The pose estimator then
 * merges these predicted poses with what the actual odometry on
 * the robot is doing and produces a smoothed "true" pose.
 */
public class UpdateVisionPose extends Command {
    PhotonVision    cameraSubsystem;
    DriveBase       robotDrive;

    /**
     * updates the odometry pose estimator to include sighted AprilTag positions from
     * PhotonVision pose estimator
     * @param cameraSubsystem the PhotonVision subsystem in use
     * @param robotDrive the drive base
     */
    public UpdateVisionPose(PhotonVision cameraSubsystem, DriveBase robotDrive) {
        this.cameraSubsystem = cameraSubsystem;
        this.robotDrive = robotDrive;

        // require camera subsystem.
        addRequirements(cameraSubsystem);
    }

    @Override
    public void initialize() {
        Util.consoleLog();
    }

    @Override
    public void execute() {
        if (Robot.isSimulation()) {
            cameraSubsystem.updateSimRobotPose(robotDrive.getPose());
            return;
        }

        Optional<EstimatedRobotPose> estimatedPoseOptional = cameraSubsystem.getEstimatedPose();

        // update pose estimator pose with current epoch timestamp and the pose from the camera
        // if the camera has a good pose output.
        // Logic to decide if a pose is valid should be put in PhotonVision.java file, not here.

        if (estimatedPoseOptional.isPresent()) {
            EstimatedRobotPose estimatedPoseContainer = estimatedPoseOptional.get();
             // convert a pose3d to pose2d (we ignore the Z axis which is just height off ground)
            Pose2d pose2d = new Pose2d(
                estimatedPoseContainer.estimatedPose.getX(),
                estimatedPoseContainer.estimatedPose.getY(),
                new Rotation2d(estimatedPoseContainer.estimatedPose.getRotation().getAngle())
            );

            robotDrive.updateOdometryVision(pose2d, estimatedPoseContainer.timestampSeconds);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
    }
}
