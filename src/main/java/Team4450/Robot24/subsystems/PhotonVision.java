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
import Team4450.Robot24.AdvantageScope;
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
 * A class that wraps a single camera connected to the
 * PhotonVision system running on a coprocessor
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
    private PhotonCameraSim         cameraSim;

    private Field2d                 field = new Field2d();

    // change the field layout for other years!
    private final AprilTagFields    fields = AprilTagFields.k2024Crescendo;
    private AprilTagFieldLayout     fieldLayout;
    private PhotonPoseEstimator     poseEstimator;

    private Transform3d             robotToCam;
    private PipelineType            pipelineType;

    public static enum PipelineType {APRILTAG_TRACKING, OBJECT_TRACKING, POSE_ESTIMATION};

    /**
     * Create an instance of PhotonVision class for a camera with a default transform. (One per camera)
     * @param cameraName the name in PhotonVision used for the camera like HD_USB_Camera
     *                   (likely from manufacturer, best not to change it to avoid conflict issues -cole)
     * @param pipelineType the PipelineType of what it's going to be used for
     */
    public PhotonVision(String cameraName, PipelineType pipelineType) {
        this(cameraName, pipelineType, new Transform3d());
    }

    public Transform3d getRobotToCam() {
        return robotToCam;
    }

    /**
     * Create an instance of PhotonVision class for a camera with a default transform. (One per camera)
     * @param cameraName the name in PhotonVision used for the camera like HD_USB_Camera
     *                   (likely from manufacturer, best not to change it to avoid conflict issues -cole)
     * @param pipelineType the PipelineType of what it's going to be used for
     * @param robotToCam a Tranformation3d of the camera relative to the bottom center of the robot (off floor).
     */
	public PhotonVision(String cameraName, PipelineType pipelineType, Transform3d robotToCam)
	{
        camera = new PhotonCamera(cameraName);
        this.robotToCam = robotToCam;
        fieldLayout = fields.loadAprilTagLayoutField();

        // adds a simulated camera to the vision sim: "real" camera will
        // act just like normal on real robot and in sim! ask cole on slack if this isn't working
        // you can manually change the 680x680 resolution and FOV
        if (RobotBase.isSimulation()) {
            visionSim = new VisionSystemSim(cameraName);
            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(680, 680, Rotation2d.fromDegrees(100)); // resolution, FOV
            this.cameraSim = new PhotonCameraSim(camera, cameraProp);
            cameraSim.enableDrawWireframe(true); // to simulate camera view
            visionSim.addCamera(cameraSim, robotToCam);
        }

        // added to support switching between multiple pipelines on one camera, but we aren't really using it.
        // regardless: must be called so that private class field `pipelineType` is set.
        selectPipeline(pipelineType);

        if (RobotBase.isSimulation()) setUpSimTargets(); // Must be after pipeline selection.

        if (pipelineType == PipelineType.POSE_ESTIMATION) {
            // setup the AprilTag pose etimator.
            poseEstimator = new PhotonPoseEstimator(
                fieldLayout, // feed in the current year's field layout
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // best one as far as we can tell
                camera,
                robotToCam
            );
        }

        setLedMode(ledMode);

		Util.consoleLog("PhotonVision (%s) created!", cameraName);
        SmartDashboard.putData(field);
	}

    /**
     * adjust the simulated camera angle for use in the simulator
     * (useful if camera is on moving mechanism such as the pre-PNW 2024 shooter)
     * @param roll roll (landscape/portrait/etc) in radians
     * @param pitch pitch (up/down) in radians
     * @param yaw yaw (left/right) in radians
     */
    public void adjustSimCameraAngle(double roll, double pitch, double yaw) {
        visionSim.adjustCamera(cameraSim, new Transform3d(
        robotToCam.getX(), robotToCam.getY(), robotToCam.getZ(), new Rotation3d(roll, pitch, yaw)));
    }

    /**
     * sets up simulation targets for simulated vision system, YEAR SPECIFIC CODE FOR OBJECT DETECTION!
     */
    private void setUpSimTargets() {
        visionSim.clearAprilTags();
        visionSim.clearVisionTargets();

        switch (pipelineType) {
            case APRILTAG_TRACKING:
                visionSim.addAprilTags(fieldLayout);
                break;
            
            case POSE_ESTIMATION:
                visionSim.addAprilTags(fieldLayout);
                break;
            
            case OBJECT_TRACKING:
                // approximate coordinates of on-field game pieces
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
                // test note:
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
        // we use cuboids to represent Notes because it's close enough and easier
        TargetModel noteModel = new TargetModel(0.3556, 0.3556, 0.0508); // approx size of note in meters
        
        VisionTargetSim target = new VisionTargetSim(
            new Pose3d(new Pose2d(x, y, new Rotation2d())), // no Z makes them kind of in floor but oh well
            noteModel
        );

        // instead of giving them one "type" we give them a unique "type" to re-access them later
        // because order of type list appears to be indeterminant so it's kind of a hacky solution but it works!
        visionSim.addVisionTargets("note"+Integer.toString(id), target);
    }

    @Override
    public void simulationPeriodic() {
        if (pipelineType == PipelineType.OBJECT_TRACKING) {
            // this stuff allows us to drag around the note in simgui
            // to change position of the note
            for (int noteID = 0; noteID < 11; noteID++) { // for each note do this:
                String name = "note" + Integer.toString(noteID); // get the unique "type"
                Pose2d fieldPose = visionSim.getDebugField().getObject(name).getPose();
                
                if (AdvantageScope.getInstance().isReservedGamepiece(noteID)) {
                    // this just means it's picked up by the robot or something else is controlling it
                    // visionSim.removeVisionTargets(name);
                } else {
                    // get pose from field and set the note pose to that so it doesn't reset
                    if (fieldPose.getX() == 0 && fieldPose.getY() == 0) continue;
                    
                    Pose3d pose3d = new Pose3d(fieldPose);
                    visionSim.getVisionTargets(name).forEach((target)->target.setPose(pose3d));
                    AdvantageScope.getInstance().setGamepiecePose(noteID, pose3d);
                }
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
     * Whether the camera is used for pose estimation or just normal apriltags
     * @return true if apriltag/pose est., false if object detection or reflective tape or other
     */
    public boolean isAprilTag() {
        return pipelineType != PipelineType.OBJECT_TRACKING;
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
    @Deprecated
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
                // loop through all the targets until the id matches
                PhotonTrackedTarget target = targets.get(i);
                if (target.getFiducialId() == id) return target;
            }

            return null;
        }
        else
            return null;
    }

    /**
     * returns the closes target to center of camera crosshair (pitch-wise)
     * @return the raw PhotonTrackedTarget
     */
    public PhotonTrackedTarget getClosestTarget() {
        PhotonTrackedTarget closest; // will hold the current closest for replacement or return

        if (hasTargets()) {
            List<PhotonTrackedTarget> targets = latestResult.getTargets();
            closest = targets.get(0); // start with first target

            for (int i = 0; i < targets.size(); i++) {
                if (targets.get(i).getPitch() < closest.getPitch()) // compare picth to closest
                    // if it's closer that closest, replace closest with it!
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
        pipelineType = type;

        if (RobotBase.isSimulation()) setUpSimTargets();
        // selectPipeline(type.ordinal());
    }

    public Pose2d getTagPose(int id) {
        Optional<Pose3d> pose3dOptional = fieldLayout.getTagPose(id);
        
        if (pose3dOptional.isPresent()) {
            Pose3d pose3d = pose3dOptional.get();
            return new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d());
        }
        else return new Pose2d();
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
        if (!isAprilTag()) {
            return Optional.empty();
        }

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
            ArrayList<Pose3d> usedTagPoses = new ArrayList<Pose3d>();
            
            for (int i = 0; i < estimatedPose.targetsUsed.size(); i++) {
                int id = estimatedPose.targetsUsed.get(i).getFiducialId();
                // if a target was used with ID > 16 then return no estimated pose
                if (id > 16) {
                    return Optional.empty();
                }
                
                Optional<Pose3d> tagPose = fieldLayout.getTagPose(id);
                
                if (tagPose.isPresent())
                    usedTagPoses.add(tagPose.get());
            }
            
            // send the tag poses used to AS to show green laser indicators of tag sights
            AdvantageScope.getInstance().setVisionTargets(usedTagPoses);
            // Util.consoleLog("used %d tags for estimation", usedTagPoses.size());

            return Optional.of(estimatedPose);
        } else {
            AdvantageScope.getInstance().setVisionTargets(new ArrayList<Pose3d>());
            return Optional.empty();
        }
    }
}