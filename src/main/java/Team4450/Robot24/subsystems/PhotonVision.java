package Team4450.Robot24.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A class that wraps the PhotonVision system running on a coprocessor
 * and adds utility functions to support using vision.
 * Note: Communication between this class and the PV on the coprocessor
 * is handled through the Network Tables and the tables are wrapped by
 * by PhotonLib.
 */
public class PhotonVision extends SubsystemBase
{
    private PhotonCamera            camera = new PhotonCamera("4450-LL");
    private PhotonPipelineResult    latestResult;
    private VisionLEDMode           ledMode = VisionLEDMode.kOff;

    private Field2d                 field = new Field2d();

    // adams code ==========
    private final AprilTagFields    fields = AprilTagFields.k2024Crescendo;
    private AprilTagFieldLayout     fieldLayout;
    private PhotonPoseEstimator     poseEstimator;

    // end adams code=============

	public PhotonVision() 
	{
        fieldLayout = fields.loadAprilTagLayoutField();

        // setup the AprilTag pose etimator
        
        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // strategy to use for tag to pose calculation
            camera, // the PhotonCamera

            // a series of transformations from Camera pos. to robot pos. (where camera is on robot)
            new Transform3d(
                new Translation3d(0,0,0.3), // approximate height // TODO: change these values to be correct - Cole W.
                new Rotation3d(0,0,Math.PI) // in radians pi is 180deg from front
            )
        );

        setLedMode(ledMode);

		Util.consoleLog("PhotonVision created!");

        // This sim field2d shows vision estimate of robot position.

        SmartDashboard.putData(field);
	}

    /**
     * Get the lastest target results object returned by the camera.
     * @return Results object.
     */
    public PhotonPipelineResult getLatestResult()
    {
        latestResult = camera.getLatestResult();

        return latestResult;
    }

    /**
     * Indicates if lastest camera results list contains targets. Must 
     * call getLatestResult() before calling.
     * @return True if targets available, false if not.
     */
    public boolean hasTargets()
    {
        getLatestResult();

        return latestResult.hasTargets();
    }

    /**
     * Returns the target with the given Fiducial ID
     * @param id the desired Fiducial ID
     * @return the target or null if the ID is not currently being tracked
     */
    public PhotonTrackedTarget getTarget(int id)
    {
        if (hasTargets()) {
            List<PhotonTrackedTarget> targets = latestResult.getTargets();

            for (int i = 0; i < targets.size(); i++) {
                PhotonTrackedTarget target = targets.get(i);
                if (target.getFiducialId() == id) return target;
            }

            return null;
        }
        else
            return null;
    }
    
    /**
     * Get an array of the currently tracked Fiducial IDs
     * 
     * @return an ArrayList of the tracked IDs
     */
    public ArrayList<Integer> getTrackedIDs() {
        ArrayList<Integer> ids = new ArrayList<Integer>();

        if (hasTargets()) {
            List<PhotonTrackedTarget> targets = latestResult.getTargets();

            for (int i=0;i<targets.size();i++) {
                ids.add(targets.get(i).getFiducialId());
            }
        }

        return ids;
    }

    /**
     * Checks whether or not the camera currently sees a target
     * with the given Fiducial ID
     * 
     * @param id the Fiducial ID
     * @return whether the camera sees the ID
     */
    public boolean hasTarget(int id) {
        return getTrackedIDs().contains(id);
    }

    // Best Target Methods =============================================================

    /**
     * Returns the yaw angle of the best target in the latest camera results
     * list. Must call hasTargets() before calling this function.
     * @return Best target yaw value from straight ahead or zero. -yaw means
     * target is left of robot center.
     */
    public double getYaw()
    {
        if (hasTargets()) 
            return latestResult.getBestTarget().getYaw();
        else
            return 0;
    }

    /**
     * Returns the Fiducial ID of the current best target, you should call
     * hasTargets() first!
     * @return the ID or -1 if no targets
     */
    public int getFiducialID()
    {
        if (hasTargets()) 
            return latestResult.getBestTarget().getFiducialId();
        else
            return -1;
    }

    /**
     * Returns the area of the best target in the latest camera results
     * list. Must call hasTargets() before calling this function.
     * @return Best target area value.
     */
    public double getArea()
    {
        if (hasTargets()) 
            return latestResult.getBestTarget().getArea();
        else
            return 0;
    }
 
    // Utility Methods =============================================================

    /**
     * Select camera's image processing pipeline.
     * @param index Zero based number of desired pipeline.
     */
    public void selectPipeline(int index)
    {
        Util.consoleLog("%d", index);

        camera.setPipelineIndex(index);
    }

    /**
     * Set the LED mode.
     * @param mode Desired LED mode.
     */
    public void setLedMode(VisionLEDMode mode)
    {
        Util.consoleLog("%d", mode.value);

        camera.setLED(mode);

        ledMode = mode;
    }

    /**
     * Toggle LED mode on/off.
     */
    public void toggleLedMode()
    {
        if (ledMode == VisionLEDMode.kOff)
            ledMode = VisionLEDMode.kOn;
        else
            ledMode = VisionLEDMode.kOff;
        
        setLedMode(ledMode);
    }

    /**
     * Save pre-processed image from camera stream.
     */
    public void inputSnapshot()
    {
        Util.consoleLog();

        camera.takeInputSnapshot();
    }

    /**
     * Save post-processed image from camera stream.
     */
    public void outputSnapshot()
    {
        Util.consoleLog();

        camera.takeOutputSnapshot();
    }
        
    @Override
	public void initSendable( SendableBuilder builder )
	{
        //super.initSendable(builder);
        builder.setSmartDashboardType("Subsystem");

        builder.addBooleanProperty("has Targets", () -> hasTargets(), null);
        builder.addDoubleProperty("target yaw", () -> getYaw(), null);
        builder.addDoubleProperty("target area", () -> getArea(), null);
	}
    
    /**
     * returns an Optional value of the robot's estimated 
     * field-centric pose given current tags that it sees.
     * (and also the timestamp)
     * 
     * @return the Optional estimated pose (empty optional means no pose or uncertain/bad pose)
     */
    public Optional<EstimatedRobotPose> getEstimatedPose() {
        Optional<EstimatedRobotPose> estimatedPoseOptional = poseEstimator.update();

        if (estimatedPoseOptional.isPresent()) {
            EstimatedRobotPose estimatedPose = estimatedPoseOptional.get();
            Pose3d pose = estimatedPose.estimatedPose;

            // pose2d to pose3d (ignore the Z axis which is height off ground)
            Pose2d pose2d = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getAngle()));

            // update the field2d object in NetworkTables to visualize where the camera thinks it's at
            field.setRobotPose(pose2d);

            // logic for checking if pose is valid would go here:
            // for example:
            for (int i = 0; i < estimatedPose.targetsUsed.size(); i++) {
                // if a target was used with ID > 16 then return no estimated pose
                if (estimatedPose.targetsUsed.get(i).getFiducialId() > 16) {
                    return Optional.empty();
                }
            }

            return Optional.of(estimatedPose);
        } else return Optional.empty();
    }
}