package Team4450.Robot24;

import java.util.ArrayList;

import Team4450.Robot24.subsystems.MAXSwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AdvantageScope {
    /** the AdvantageScope singleton */
    private static AdvantageScope instance;
    private double elevHeight = 0;
    private double shooterAngle = 0;

    /** the robot pose (2D) */
    private Pose2d robotPose = new Pose2d();

    /** an array of the IDs of gamepieces the robot is currently "holding" */
    private ArrayList<Integer> gamepieceInventory = new ArrayList<Integer>();

    /** an array of the poses of all gamepieces on field */
    private Pose3d[] gamepieces = new Pose3d[11];

    /** an array of swerve states (angle, velocity, angle, velocity, etc.) */
    private double[] swerveStates = new double[8];

    /** the gyro angle of the robot */
    private double gyro = 0;

    /** an array of currently sighted vision target poses */
    private ArrayList<Pose3d> visionTargets = new ArrayList<Pose3d>();

    public void setElevatorHeight(double h) {elevHeight = h;}
    public void setShooterAngle(double a) {shooterAngle = -a;}

    /**
     * Set the pose of a given gamepiece
     * @param id the gamepiece ID
     * @param pose the Pose3d of the gamepiece
     */
    public void setGamepiecePose(int id, Pose3d pose) {
        gamepieces[id] = pose;
    }

    /**
     * Update the mechanism poses and component and robot positions
     */
    public void update() {
        // season specific mechanism values:
        Pose3d elevatorPose = new Pose3d(0, 0, elevHeight, new Rotation3d());
        Pose3d shooterPose = new Pose3d(0.09, 0, 0.275 + elevHeight, new Rotation3d(0, Math.toRadians(-25 + shooterAngle), 0));
        Pose3d carriagePose = new Pose3d(0, 0, elevHeight + 0.08, new Rotation3d());

        // send components
        sendPoses("components", elevatorPose, shooterPose, carriagePose);
        sendPoses("robot", new Pose3d(robotPose));
        sendPoses("notes", gamepieces);
        sendPoses("targets", visionTargets.toArray(new Pose3d[0]));

        SmartDashboard.putNumberArray("Visualization/swerve_modules", swerveStates);
        SmartDashboard.putNumber("Visualization/gyro", gyro);

        // for all the notes in the gamepiece inventory, set the pose to be above the robot
        // this could be changed to make it look like they are "in" the robot but thats not
        // super necessary (it would be cool though)
        for (int i=0;i<gamepieceInventory.size();i++) {
            int id = gamepieceInventory.get(i);
            setGamepiecePose(id, new Pose3d(robotPose.getX(), robotPose.getY(), 0.5, new Rotation3d()));
        }
    }

    /**
     * Set the list of currently sighted vision target poses for use in AdvantageScope
     * @param targets
     */
    public void setVisionTargets(ArrayList<Pose3d> targets) {
        visionTargets = targets;
    }

    /**
     * Deconstruct the Pose3d to a numerical quaternion array
     * @param pose the Pose3d
     * @return a double array of length 7 containing the quaternion values
     */
    private double[] poseToArray(Pose3d pose) {
        if (pose == null) {
            double[] empty = {0,0,0,0,0,0,0};
            return empty;
        }

        Quaternion quat = pose.getRotation().getQuaternion();
        double[] array = {pose.getX(), pose.getY(), pose.getZ(), quat.getW(), quat.getX(), quat.getY(), quat.getZ()};
        return array;
    }

    /**
     * Send one or more Pose3ds with the given key to NetworkTables by deconstructing them into a numerical array
     * of quaternion values for use in AdvantageScope
     * @param key the name to put under the "Visualization/" subheading
     * @param poses one or more Pose3ds
     */
    public void sendPoses(String key, Pose3d... poses) {
        ArrayList<Double> output = new ArrayList<Double>();
        
        for (int i=0;i<poses.length;i++) {
            double[] poseArray = poseToArray(poses[i]);
            
            for (int j=0;j<poseArray.length;j++) output.add(poseArray[j]);
        }
        
        Double[] outputArray = output.toArray(new Double[0]);
        SmartDashboard.putNumberArray("Visualization/"+key, outputArray);
    }

    /**
     * Pickup the gamepiece with the given ID, adding it to the robot's
     * "inventory"
     * @param id the gamepiece ID
     */
    public void pickupGamepiece(int id) {
        if (!gamepieceInventory.contains(id)) gamepieceInventory.add(id);
    }

    /**
     * Attempt to pick up a gamepiece if it is within 0.5 meters of the center of the robot,
     * adding it to the "reserved" list.
     * @return whether there was a successful pickup (for simulation)
     */
    public boolean attemptPickup() {
        for (int i=0;i<gamepieces.length;i++) {
            Pose3d note = gamepieces[i];
            
            if (note == null) return true;
            
            double xdist = Math.abs(note.getX() - robotPose.getX());
            double ydist = Math.abs(note.getY() - robotPose.getY());
            double dist = Math.sqrt(Math.pow(xdist, 2) + Math.pow(ydist, 2));
            
            if (dist < 0.5) {
                pickupGamepiece(i);
                return true;
            }

        }
        return false;
    }

    /**
     * Checks if the robot has at least one gamepiece (anything in "inventory").
     * Useful for beambreak simulation, etc.
     * @return true/false
     */
    public boolean hasAGamepiece() {
        if (gamepieceInventory.size() > 0 && RobotBase.isSimulation()) return true;
        else return false;
    }

    /**
     * Clear or "drop" the given gamepiece
     * @param id the ID of the gamepiece to clear
     */
    public void clearInventoryGamepiece(int id) {
        gamepieceInventory.remove(id);
    }

    /**
     * Clears the cache of reserved gamepieces ("dropping" them)
     */
    public void clearGamepieceInventory() {
        gamepieceInventory.clear();
    }


    /**
     * Check if the gamepiece with the given ID is reserved (in robot, etc.)
     * @param id the gamepiece ID
     * @return true or false
     */
    public boolean isReservedGamepiece(int id) {
        return gamepieceInventory.contains(id);
    }

    /**
     * Set the fused odometry/vision pose in AdvantageScope
     * @param pose the 2D fused pose from a SwervePoseEstimator
     */
    public void setRobotPose(Pose2d pose) {
        robotPose = pose;
        gyro = pose.getRotation().getDegrees();
    }

    // FL, FR, BL, BR
    /**
     * Sends the swerve module poses to AdvantageScope
     * @param fl the Front Left MAXSwerveModule object
     * @param fr the Front Right MAXSwerveModule object
     * @param bl the Back Left MAXSwerveModule object
     * @param br the Back Right MAXSwerveModule object
     */
    public void setSwerveModules(MAXSwerveModule fl, MAXSwerveModule fr, MAXSwerveModule bl, MAXSwerveModule br) {
        swerveStates[0] = fl.getState().angle.getDegrees();
        swerveStates[1] = fl.getState().speedMetersPerSecond;
        swerveStates[2] = fr.getState().angle.getDegrees();
        swerveStates[3] = fr.getState().speedMetersPerSecond;
        swerveStates[4] = bl.getState().angle.getDegrees();
        swerveStates[5] = bl.getState().speedMetersPerSecond;
        swerveStates[6] = br.getState().angle.getDegrees();
        swerveStates[7] = br.getState().speedMetersPerSecond;
    }

    /**
     * Access the singleton AdvantageScope object for use in code
     * @return the AdvantageScope singleton
     */
    public static AdvantageScope getInstance() {
        if (AdvantageScope.instance == null) AdvantageScope.instance = new AdvantageScope();
        
        return AdvantageScope.instance;
    }
}
