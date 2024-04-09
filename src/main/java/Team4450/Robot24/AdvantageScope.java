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
    private static AdvantageScope instance;
    private double elevHeight = 0;
    private double shooterAngle = 0;
    private Pose2d robotPose = new Pose2d();

    private ArrayList<Integer> reservedNotes = new ArrayList<Integer>();
    // private double[] notes = new double[7 * 11];
    private Pose3d[] notes = new Pose3d[11];
    private double[] swerveStates = new double[8];
    private double gyro = 0;
    private ArrayList<Pose3d> visionTargets = new ArrayList<Pose3d>();

    public AdvantageScope() {
    }

    public void setElevatorHeight(double h) {
        elevHeight = h;
    }

    public void setCarriageHeight(double h) {
        // cHeight = h;
    }

    public void setShooterAngle(double a) {
        shooterAngle = -a;
    }

    public void setNote(int id, Pose3d pose) {
        notes[id] = pose;
    }

    public void update() {
        Pose3d elevatorPose = new Pose3d(0, 0, elevHeight, new Rotation3d());
        Pose3d shooterPose = new Pose3d(0.09, 0, 0.275+elevHeight, new Rotation3d(0, Math.toRadians(-25 + shooterAngle), 0));
        Pose3d carriagePose = new Pose3d(0, 0, elevHeight+0.08, new Rotation3d());
        sendPoses("components", elevatorPose, shooterPose, carriagePose);

        sendPoses("robot", new Pose3d(robotPose));
        sendPoses("notes", notes);
        sendPoses("targets", visionTargets.toArray(new Pose3d[0]));

        SmartDashboard.putNumberArray("Visualization/swerve_modules", swerveStates);
        SmartDashboard.putNumber("Visualization/gyro", gyro);

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

    public void sendPoses(String key, Pose3d... poses) {
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
            if (note == null) return true;
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

    public boolean hasNoteSim() {
        if (reservedNotes.size() > 0 && RobotBase.isSimulation()) return true;
        else return false;
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
        gyro = pose.getRotation().getDegrees();
    }

    // FL, FR, BL, BR
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

    public static AdvantageScope getInstance() {
        if (AdvantageScope.instance == null)
            AdvantageScope.instance = new AdvantageScope();
        return AdvantageScope.instance;
    }
}
