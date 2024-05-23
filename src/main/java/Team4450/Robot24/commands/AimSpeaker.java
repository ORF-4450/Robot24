package Team4450.Robot24.commands;

import static Team4450.Robot24.Constants.SUBWOOFER_ANGLE;
import static Team4450.Robot24.Constants.alliance;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.PhotonVision;
import Team4450.Robot24.utility.AprilTagNames;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Aims the shooter and drivebase at the current alliances' speaker
 * using vision and linear angle interpolation based on distance values.
 * This command also spins the shooter wheels, although doesn't move to any
 * initial position (which must be done elsewhere to safely pivot!).
 */
public class AimSpeaker extends Command {
    private final DriveBase robotDrive;
    private final ElevatedShooter elevatedShooter;
    private final PhotonVision photonVision;
    private final DoubleSupplier joystick;
    // private double lastSight = 0;
    private AprilTagNames tagNames = new AprilTagNames(alliance); // helper class for tag names
    public static enum Position {LOW, NORMAL, HIGH}

    private final PIDController rotationController = new PIDController(0.02, 0, 0); // for rotating drivebase

    // these "InterpolatingDoubleTreeMap"s allow us to put in several data points and get a "best guess"
    // estimate for points that don't exist using the neighboring points, pretty useful so thanks WPILib!
    private final InterpolatingDoubleTreeMap pitchOffsets = new InterpolatingDoubleTreeMap(); // distance -> shooter angle mapping
    private final InterpolatingDoubleTreeMap yawOffsets = new InterpolatingDoubleTreeMap(); // distance -> robot yaw offset mapping

    /**
     * Aims robot base and shooter pivot at the current alliance speaker using
     * vision and interpolated distance values, while still allowing driver override.
     *
     * @apiNote NOTE! This command doesn't actually use this output to directly control robot
     * rotation, but instead uses it to check if joystick is being moved and passes off control
     * to the normal drive command if joystick is being moved.
     * 
     * @param robotDrive the drivebase subsystem
     * @param elevatedShooter the ElevatedShooter conglomerate instance
     * @param photonVision the camera to use
     * @param joystick a supplier for the rotation joystick value (see above note)
     * @param upperShot whether to use the pitch offsets for a low or high shot
     */
    public AimSpeaker(DriveBase robotDrive, ElevatedShooter elevatedShooter, PhotonVision photonVision, DoubleSupplier joystick, Position position) {
        this.robotDrive = robotDrive;
        this.elevatedShooter = elevatedShooter;
        this.photonVision = photonVision;
        this.joystick = joystick;

        // We don't require drivebase here because we still want the main drive command to
        // have control, and we don't need complete control over it. The camera should never
        // be required because its default command is used to update odometry.
        addRequirements(elevatedShooter);

        rotationController.setTolerance(0.3); // degrees
        SendableRegistry.addLW(rotationController, "AimSpeaker Rotation PID");

        // mapping of distance (meters) to shooter angle
        // as a reminder, 0deg is horizontal, and angles are CCW positive
        // referencing the infeed side of the shooter. So an angle of -45 is
        // shooter pointing up and infeed rollers 45deg below horizontal
        switch (position) {
            case LOW:
                // NOT USING
                break;

            case NORMAL:
                // NORMAL SHOT POSITION
                // (distance, angle) (add decimals or .0 after all)
                // pitchOffsets.put(1.63556, -53.86);
                // pitchOffsets.put(2.309, -43.224);
                // pitchOffsets.put(3.29, -34.63);
                // pitchOffsets.put(4.92, -27.50);
                // pitchOffsets.put(5.56, -25.89);
                // pitchOffsets.put(6.26, -25.0);
                //NEW CAMERA MOUNT BELOW:
                // pitchOffsets.put(2.01,-52.26);
                // pitchOffsets.put(2.97,-44.83);
                // pitchOffsets.put(3.30,-41.04);
                // pitchOffsets.put(3.79,-36.67);
                // pitchOffsets.put(4.31,-32.45);
                // pitchOffsets.put(4.35,-31.86);
                // pitchOffsets.put(4.50,-30.11);
                // pitchOffsets.put(6.36,-22.69);
                // pitchOffsets.put(7.75,-20.06);
                // pitchOffsets.put(9.61,-18.32);
                // pitchOffsets.put(11.6,-18.61);

                // EOD thursday
                pitchOffsets.put(0.05 + 1.68, SUBWOOFER_ANGLE);
                pitchOffsets.put(0.05 + 1.76, -51.38 + 0.05);
                pitchOffsets.put(0.05 + 2.13,-46.80 + 0.05);
                pitchOffsets.put(0.05 + 2.48, -38.71 - 0.25);
                pitchOffsets.put(0.05 + 3.2278, -31.44);
                pitchOffsets.put(0.05 + 3.46, -28.97);
                pitchOffsets.put(0.05 + 3.59, -27.2);
                pitchOffsets.put(0.05 + 3.62, -28.53);
                pitchOffsets.put(0.05 + 3.81, -26.76);
                // pitchOffsets.put(4.01, -27.9);
                pitchOffsets.put(4.03, -26.4);
                pitchOffsets.put(4.34, -25.33);
                pitchOffsets.put(4.95, -22.12);
                pitchOffsets.put(5.72584, -20.21);             
                break;

            case HIGH:
                // HIGH SHOT POSITION
                // (distance, angle)
                // pitchOffsets.put(4.72, -21.81);
                // pitchOffsets.put(6.37, -18.0);
                // pitchOffsets.put(4.03, -25.3);
                // pitchOffsets.put(7.96, -15.0);
                // pitchOffsets.put(6.01, -19.48);
                //NEW CAMERA MOUNT BELOW:

                pitchOffsets.put(2.02, -42.50);
                pitchOffsets.put(3.36, -26.04);
                pitchOffsets.put(4.48, -23.71);
                pitchOffsets.put(5.10, -21.96);
                pitchOffsets.put(6.18, -16.80);
                pitchOffsets.put(6.98, -16.27);
                pitchOffsets.put(8.89, -15.40);
                pitchOffsets.put(5.72, -20.79);
                pitchOffsets.put(4.23, -25.02);
                pitchOffsets.put(3.44, -27.78);
                break;
        }
        
        // mapping of distance (meters) to target yaw offset value within frame
        // Ideally, the yaw offset should be 0 deg because that is
        // perfectly centered within PV frame, but that's only if the camera is on
        // the center of the robot. CHANGE: it is now so it's just 0 for everything:
        yawOffsets.put(0.0, 0.0);
        yawOffsets.put(15.0, 0.0);
        // yawOffsets.put(20.0, 6.0);

        // this indicator should represent whether the shooter AND the drivebase are in
        // alignment AND ready to shoot.
        SmartDashboard.putBoolean("Target Locked", false);
    }

    @Override
    public void initialize() {
        Util.consoleLog();

        tagNames = new AprilTagNames(alliance);
        // this.lastSight = 0;
        
        SmartDashboard.putBoolean("Target Locked", false);

        // enable the "tracking" parameter so that when set,
        // the drivebase ignores the joysticks and rotates on its own.
        // we always want to make sure to turn this off and not leave it
        // on when the command ends
        robotDrive.enableTracking();
    }

    @Override
    public void execute() {
        double currentAngle = elevatedShooter.shooter.getAngle();
        double joystickValue = joystick.getAsDouble(); // check if joystick is moving
        boolean joystickMoving = joystickValue < -0.03 || joystickValue > 0.03;

        // if we had a camera actually mounted on the shooter itself, we would want
        // to update the simulation camera to reflect movement of the shooter. However
        // we aren't doing this anymore but I wanted to leave this commented out as an example. - Cole
        //      if (RobotBase.isSimulation()) shooterCamera.adjustSimCameraAngle(0, Math.toRadians(currentAngle), Math.toRadians(180));

        int targetId = tagNames.SPEAKER_MAIN; // the apriltag to target (switches ID based on alliance)
        PhotonTrackedTarget target = photonVision.getTarget(targetId);

        if (target == null ) {
            Util.consoleLog("null target");
            SmartDashboard.putBoolean("Target Locked", false);
            SmartDashboard.putNumber("Distance to Speaker", Double.NaN);

            // Transform2d transform = robotDrive.getPose().minus(photonVision.getTagPose(targetId));
            // double idealYaw = Math.toDegrees(Math.atan2(transform.getY(), transform.getX()));
            // double currentYaw = robotDrive.getGyroYaw();

            // if (joystickMoving || Util.getElaspedTime(lastSight) < 2) {robotDrive.setTrackingRotation(Double.NaN);} // if so, just do joystick
            // else {
            //     // if joystick not moving, use PID to attempt to match the yaw offset interpolated using above values
            //     // at the current distance
            //     Util.consoleLog("id = %d", targetId);
            //     double output = rotationController.calculate(idealYaw - currentYaw, 0);
            //     robotDrive.setTrackingRotation(output); // will be handled in drivebase as "faked" joystick input
            // }

            robotDrive.setTrackingRotation(Double.NaN);
            robotDrive.clearPPRotationOverride();
            return;
        }
        // lastSight = Util.timeStamp();
        // else we just continue as is knowing target is not null

        boolean yawOkay = false;
        boolean pitchOkay = false;

        // calculate distance to tag using PhotonUtils (basic trig)
        double dist = PhotonUtils.calculateDistanceToTargetMeters(photonVision.getRobotToCam().getZ(), 1.447, // camera height, tag height (center)
            -photonVision.getRobotToCam().getRotation().getY(),
            Math.toRadians(target.getPitch())
        );

        // double pitchAngle = pitchOffsets.get(dist);
        double pitchAngle = pitchOffsets.get(dist);
        double yawAngle = yawOffsets.get(dist);

        double[] velocityOffsets = getMovementOffsets(pitchAngle);
        yawAngle += 0.7 * velocityOffsets[0];
        pitchAngle += 1.0 * velocityOffsets[1];

        elevatedShooter.shooter.startShooting(1.0 * velocityOffsets[2]); // keep the wheels spun up!

        // ============= rotation of robot ==============================
        // points the physical drivebase at the speaker using yaw offsets

        if (joystickMoving) {
            robotDrive.setTrackingRotation(Double.NaN);
            robotDrive.clearPPRotationOverride();
        } // if so, just do joystick
        else {
            // if joystick not moving, use PID to attempt to match the yaw offset interpolated using above values
            // at the current distance
            double output = rotationController.calculate(target.getYaw(), yawAngle);
            
            if (RobotState.isAutonomous()) {
                robotDrive.setPPRotationOverrideOffset(target.getYaw());
            }

            if (Math.abs(output) < 0.02) yawOkay = true; // if within tolerance than the yaw is good
            
            robotDrive.setTrackingRotation(output); // will be handled in drivebase as "faked" joystick input
        }

        // ============ shooter pivot ==================================
        // pivots the shooter to the interpolated angle
        //
        // NOTE: at intake position it's not safe to pivot,
        // another command must first raise elevator a little
        double setpoint = pitchAngle; // degrees
        setpoint = Util.clampValue(setpoint, -90, 0);
        elevatedShooter.shooter.setAngle(setpoint); // degrees
        if (Math.abs(setpoint - currentAngle) < 3) pitchOkay = true; // 3 deg tolerance

        // logging: yes it's annoying but it's also helpful
        Util.consoleLog("dist=%f angle=%f", dist, pitchOffsets.get(dist));
        // Util.consoleLog("shooter_angle=%f pitch=%f yaw=%f", currentAngle, target.getPitch(), target.getYaw());
        SmartDashboard.putBoolean("Target Locked", yawOkay && pitchOkay);
        SmartDashboard.putNumber("Distance to Speaker", dist);
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean("Target Locked", false);
        SmartDashboard.putNumber("Distance to Speaker", Double.NaN);

        elevatedShooter.shooter.stopShooting();
        robotDrive.disableTracking();
    }

    /**
     * Returns the degree and speed offsets for aiming based on the robot velocity.
     * This is because if we shoot while moving the Note keeps the inertia/momentum
     * so we need to rotate off target to make it go in.
     * @param theta the desired pitch of the shooter in degrees
     * @return an array of 3 values: the change in yaw angle, the change in pitch angle, and the change in speed needed
     */
    private double[] getMovementOffsets(double theta) {
        ChassisSpeeds chassisSpeeds = robotDrive.getChassisSpeeds();
        double shotSpeed = elevatedShooter.shooter.getWheelSpeed();
        if (RobotBase.isSimulation()) shotSpeed = 10;
        theta = -Math.toRadians(theta);

        // see https://www.desmos.com/3d/bdc5aeb804
        Vector<N3> robotSpeeds = VecBuilder.fill(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond, 0);
        Vector<N3> idealShotVector = VecBuilder.fill(0, shotSpeed*Math.cos(theta), shotSpeed*Math.sin(theta));
        Vector<N3> finalShotVector = idealShotVector.plus(robotSpeeds);

        double fX = finalShotVector.get(0, 0);
        double fY = finalShotVector.get(1, 0);
        double fZ = finalShotVector.get(2, 0);
        // double fMag=Math.sqrt(Math.pow(fX,2) + Math.pow(fY,2) + Math.pow(fZ,2));

        theta = Math.toDegrees(theta);
        double alpha = -Math.toDegrees(Math.atan(fX / fY));
        double beta = Math.toDegrees(Math.atan(fZ / fY));
        double speedMultiplier = 1;//fMag / shotSpeed;

        if (RobotBase.isReal()) alpha *= -1;

        double[] output = {alpha, theta - beta, speedMultiplier};

        Util.consoleLog("d_yaw=%f, theta=%f, d_pitch=%f, m_speed=%f", alpha, theta, theta-beta, speedMultiplier);

        return output;
    }

    @Override
    public boolean isFinished() {
        // this will be set by another command
        return elevatedShooter.shooter.hasShot;
    }
}
