package Team4450.Robot24;

import static Team4450.Robot24.Constants.robot;

import java.util.ArrayList;

import java.util.Optional;

import Team4450.Lib.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AdvantageScope {
    private static AdvantageScope instance;
    private double elevHeight = 0;
    private double shooterAngle = 0;
    private double cHeight = 0;
    private Pose2d robotPose = new Pose2d();

    private ArrayList<Integer> reservedNotes = new ArrayList<Integer>();
    // private double[] notes = new double[7 * 11];
    private Pose3d[] notes = new Pose3d[11];
    private ArrayList<Pose3d> visionTargets = new ArrayList<Pose3d>();

    public AdvantageScope() {
    }

    public void setElevatorHeight(double h) {
        elevHeight = Util.clampValue(h, 0, 1);
    }

    public void setCarriageHeight(double h) {
        cHeight = Util.clampValue(h, 0, 0.5);
    }

    public void setShooterAngle(double a) {
        shooterAngle = a;
    }

    public void setNote(int id, Pose3d pose) {
        notes[id] = pose;
    }

    public void update() {
        Pose3d elevatorPose = new Pose3d(0, 0, cHeight, new Rotation3d());
        Pose3d shooterPose = new Pose3d(0.09, 0, 0.259+elevHeight, new Rotation3d(0, Math.toRadians(-25 + shooterAngle), 0));
        Pose3d carriagePose = new Pose3d(0, 0, elevHeight, new Rotation3d());
        sendPoses("components", elevatorPose, shooterPose, carriagePose);

        sendPoses("robot", new Pose3d(robotPose));
        sendPoses("notes", notes);
        sendPoses("targets", visionTargets.toArray(new Pose3d[0]));

        for (int i=0;i<reservedNotes.size();i++) {
            int id = reservedNotes.get(i);
            setNote(id, new Pose3d(robotPose.getX(), robotPose.getY(), 0.5, new Rotation3d()));
        }
    }

    public void setVisionTargets(ArrayList<Pose3d> targets) {
        visionTargets = targets;
    }

    private double[] poseToArray(Pose3d pose) {
        if (pose == null) {
            double[] empty = {0,0,0,0,0,0,0};
            return empty;
        }
        Quaternion quat = pose.getRotation().getQuaternion();
        double[] array = {pose.getX(), pose.getY(), pose.getZ(), quat.getW(), quat.getX(), quat.getY(), quat.getZ()};
        return array;
    }

    private void sendPoses(String key, Pose3d... poses) {
        ArrayList<Double> output = new ArrayList<Double>();
        for (int i=0;i<poses.length;i++) {
            double[] poseArray = poseToArray(poses[i]);
            for (int j=0;j<poseArray.length;j++)
                output.add(poseArray[j]);
        }
        Double[] outputArray = output.toArray(new Double[0]);
        SmartDashboard.putNumberArray("Visualization/"+key, outputArray);
    }

    public void pickupNote(int id) {
        if (!reservedNotes.contains(id))
            reservedNotes.add(id);
    }

    public boolean attemptPickup() {
        for (int i=0;i<notes.length;i++) {
            Pose3d note = notes[i];
            double xdist = Math.abs(note.getX() - robotPose.getX());
            double ydist = Math.abs(note.getY() - robotPose.getY());
            double dist = Math.sqrt(Math.pow(xdist, 2) + Math.pow(ydist, 2));
            if (dist < 0.5) {
                pickupNote(i);
                return true;
            }

        }
        return false;
    }

    public void dropNote(int id) {
        reservedNotes.remove(id);
    }

    public void dropAllNotes() {
        reservedNotes.clear();
    }

    public boolean isReserved(int id) {
        return reservedNotes.contains(id);
    }

    public void setRobotPose(Pose2d pose) {
        robotPose = pose;
    }

    public static AdvantageScope getInstance() {
        if (AdvantageScope.instance == null)
            AdvantageScope.instance = new AdvantageScope();
        return AdvantageScope.instance;
    }
}
