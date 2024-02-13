package Team4450.Robot24.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;
import Team4450.Robot24.Robot;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
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
    private PhotonCamera            camera;
    private PhotonPipelineResult    latestResult;

    private VisionLEDMode           ledMode = VisionLEDMode.kOff;

    private VisionSystemSim         visionSim;

    private Field2d                 field = new Field2d();

    private final AprilTagFields    fields = AprilTagFields.k2024Crescendo;
    private AprilTagFieldLayout     fieldLayout;
    private PhotonPoseEstimator     poseEstimator;

    private Transform3d             robotToCam;
    private PipelineType            pipelineType;

    public static enum PipelineType {APRILTAG_TRACKING, OBJECT_TRACKING};

    /**
     * Create an instance of PhotonVision class for a camera.
     * @param cameraName The name of the camera.
     */
    public PhotonVision(String cameraName, PipelineType pipelineType) {
        this(cameraName, pipelineType, new Transform3d());
    }

    /**
     * Create an instance of PhotonVision class for a camera.
     * @param cameraName The name of the camera.
     * @param robotToCam A Transform3d locating the camera on the robot chassis.
     */
	public PhotonVision(String cameraName, PipelineType pipelineType, Transform3d robotToCam)
	{
        camera = new PhotonCamera(cameraName);
        this.robotToCam = robotToCam;
        fieldLayout = fields.loadAprilTagLayoutField();

        // adds a simulated camera to the vision sim: "real" camera will
        // act just like normal on real robot and in sim!
        if (RobotBase.isSimulation()) {
            visionSim = new VisionSystemSim(cameraName);
            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
            cameraSim.enableDrawWireframe(true);
            visionSim.addCamera(cameraSim, robotToCam);
        }

        selectPipeline(pipelineType);

        if (RobotBase.isSimulation()) setUpSimTargets();    // Must follow pipeline selection.

        // setup the AprilTag pose etimator.
        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            robotToCam
        );

        setLedMode(ledMode);

		Util.consoleLog("PhotonVision (%s) created!", cameraName);
        SmartDashboard.putData(field);
	}

    /**
     * sets up simulation targets for simulated vision system
     */
    private void setUpSimTargets() {
        visionSim.clearAprilTags();
        visionSim.clearVisionTargets();

        switch (pipelineType) {
            case APRILTAG_TRACKING:
                visionSim.addAprilTags(fieldLayout);
                break;

            case OBJECT_TRACKING:
                // approximate coordinates on on-field notes
                addNoteSimTarget(2.9, 7, 0);
                addNoteSimTarget(2.9, 5.6, 1);
                addNoteSimTarget(2.9, 4.1, 2);
                addNoteSimTarget(8.3, 7.5, 3);
                addNoteSimTarget(8.3, 5.8, 4);
                addNoteSimTarget(8.3, 4.1, 5);
                addNoteSimTarget(8.3, 2.4, 6);
                addNoteSimTarget(8.3, 0.75, 7);
                addNoteSimTarget(13.65, 7, 8);
                addNoteSimTarget(13.65, 5.6, 9);
                addNoteSimTarget(13.65, 4.1, 10);

                // test note
                // addNoteSimTarget(5, 5, 10);
                break;
        }
    }

    /**
     * adds a sim target of a Note
     * @param x field coordinate X
     * @param y field coordinate Y
     * @param id zero-based index of note
     */
    private void addNoteSimTarget(double x, double y, int id) {
        TargetModel noteModel = new TargetModel(0.3556, 0.3556, 0.0508);
        
        VisionTargetSim target = new VisionTargetSim(
            new Pose3d(new Pose2d(x, y, new Rotation2d())),
            noteModel
        );
        
        visionSim.addVisionTargets("note"+Integer.toString(id), target);
    }


    @Override
    public void simulationPeriodic() {
        if (pipelineType == PipelineType.OBJECT_TRACKING) {
            // this stuff allows us to drag around the note in simgui
            // to change position of the note
            for (int noteID = 0; noteID < 11; noteID++) {
                String name = "note" + Integer.toString(noteID);
                Pose2d fieldPose = visionSim.getDebugField().getObject(name).getPose();
                if (fieldPose.getX() == 0 && fieldPose.getY() == 0) continue;
                visionSim.getVisionTargets(name).forEach((target)->target.setPose(new Pose3d(fieldPose)));
            }
        }
    }

    /**
     * Updates the simulated pose of the robot for use in the PhotonVision
     * simulated vision code.
     * @param pose The pose of robot (ONLY BASED OFF OF ODOMETRY: NOT ANY POSE ESTIMATORS).
     */
    public void updateSimulationPose(Pose2d pose) {
        visionSim.update(pose);
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
     * Returns the target with the given Fiducial ID.
     * @param id the desired Fiducial ID.
     * @return The target or null if the ID is not currently being tracked.
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
     * returns the closes target to center of camera crosshair (yaw-wise)
     * @return the raw PhotonTrackedTarget
     */
    public PhotonTrackedTarget getClosestTarget() {
        PhotonTrackedTarget closest;
        if (hasTargets()) {
            List<PhotonTrackedTarget> targets = latestResult.getTargets();
            closest = targets.get(0);

            for (int i = 0; i < targets.size(); i++) {
                if (Math.abs(targets.get(i).getYaw()) < Math.abs(closest.getYaw()))
                    closest = targets.get(i);
            }
            return closest;
        }
        else
            return null;
    }
    
    /**
     * Get an array of the currently tracked Fiducial IDs.
     * 
     * @return An ArrayList of the tracked IDs.
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
     * with the given Fiducial ID.
     * 
     * @param id The Fiducial ID.
     * @return True if the camera sees the ID.
     */
    public boolean hasTarget(int id) {
        return getTrackedIDs().contains(id);
    }

    // Best Target Methods =============================================================

    /**
     * Returns the yaw angle of the best target in the latest camera results
     * list. Must call hasTargets() before calling this function.
     * @return Best target yaw value from straight ahead or zero. -yaw means
     * target is left of robot center. (degrees)
     */
    public double getYaw()
    {
        if (hasTargets()) 
            return latestResult.getBestTarget().getYaw();
        else
            return 0;
    }

    /**
     * Returns the pitch angle of the best target in the latest camera results
     * list. Must call hasTargets() before calling this function.
     * @return Best target pitch value from straight ahead or zero. -pitch means
     * target is below camera center. (degrees)
     */
    public double getPitch()
    {
        if (hasTargets()) 
            return latestResult.getBestTarget().getPitch();
        else
            return 0;
    }

    /**
     * Returns the Fiducial ID of the current best target, you should call
     * hasTargets() first!
     * @return The ID or -1 if no targets.
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
     * Sets the pipline of the photonvision camera given an enum type.
     * This also allows us to use simulation pretty well!
     * @param type The type of pipeline.
     */
    public void selectPipeline(PipelineType type) {
        this.pipelineType = type;
        if (RobotBase.isSimulation())
            setUpSimTargets();
        selectPipeline(type.ordinal());
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
        builder.addDoubleProperty("target pitch", () -> getPitch(), null);
        builder.addDoubleProperty("target area", () -> getArea(), null);
	}
    
    /**
     * Returns an Optional value of the robot's estimated 
     * field-centric pose given current tags that it sees.
     * (and also the timestamp)
     * 
     * @return The Optional estimated pose (empty optional means no pose or uncertain/bad pose).
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