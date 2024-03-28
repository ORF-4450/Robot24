package Team4450.Robot24.commands;

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
    private final AprilTagNames tagNames = new AprilTagNames(alliance); // helper class for tag names

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
    public AimSpeaker(DriveBase robotDrive, ElevatedShooter elevatedShooter, PhotonVision photonVision, DoubleSupplier joystick, boolean upperShot) {
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
        if (upperShot) {
            // UPPER SHOT POSITION
            pitchOffsets.put(1.45, -44.0);
            pitchOffsets.put(2.37, -33.0);
            pitchOffsets.put(3.45, -20.0);
            pitchOffsets.put(3.69, -16.0);
        } else {
            // LOWER SHOT POSITION
            pitchOffsets.put(1.69, -55.0);
            pitchOffsets.put(1.77, -48.0);
            pitchOffsets.put(2.10, -42.06);
            pitchOffsets.put(2.53, -35.07);
            pitchOffsets.put(2.87, -30.26);
            pitchOffsets.put(3.28, -27.78);
            pitchOffsets.put(3.53, -26.32);
            pitchOffsets.put(4.57, -19.92);
        }
        
    
        // mapping of distance (meters) to target yaw offset value within frame
        // Ideally, the yaw offset should be 0 deg because that is
        // perfectly centered within PV frame, but that's only if the camera is on
        // the center of the robot. If it is on the side we have to "center" a little
        // bit off to the side:
        yawOffsets.put(1.69, 9.0);
        yawOffsets.put(20.0, 6.0);

        // this indicator should represent whether the shooter AND the drivebase are in
        // alignment AND ready to shoot.
        SmartDashboard.putBoolean("Target Locked", false);
    }

    @Override
    public void initialize() {
        Util.consoleLog();
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

            // here we don't turn off tracking mode, but temporarily put NaN which disables
            // it temporarily. no real difference then disabling it but then we don't have to re-enable
            // so it's just a little easier
            robotDrive.setTrackingRotation(Double.NaN);
            return;
        }
        // else we just continue as is knowing target is not null

        boolean yawOkay = false;
        boolean pitchOkay = false;

        // calculate distance to tag using PhotonUtils (basic trig)
        double dist = PhotonUtils.calculateDistanceToTargetMeters(photonVision.getRobotToCam().getZ(), 1.4224, // camera height, tag height (center)
            -photonVision.getRobotToCam().getRotation().getY(),
            Math.toRadians(target.getPitch())
        );


        double pitchAngle = pitchOffsets.get(dist);
        double yawAngle = yawOffsets.get(dist);

        double[] velocityOffsets = getMovementOffsets(pitchAngle);
        yawAngle += 0.7 * velocityOffsets[0];
        pitchAngle += 1.0 * velocityOffsets[1];

        elevatedShooter.shooter.startShooting(1.0 * velocityOffsets[2]); // keep the wheels spun up!

        // ============= rotation of robot ==============================
        // points the physical drivebase at the speaker usig yaw offsets

        double joystickValue = joystick.getAsDouble(); // check if joystick is moving
        if (joystickValue < -0.03 || joystickValue > 0.03) {robotDrive.setTrackingRotation(Double.NaN);} // if so, just do joystick
        else {
            // if joystick not moving, use PID to attempt to match the yaw offset interpolated using above values
            // at the current distance
            double output = rotationController.calculate(target.getYaw(), yawAngle);
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
        SmartDashboard.putBoolean("Target Locked", false);
        SmartDashboard.putNumber("Distance to Speaker", Double.NaN);
        Util.consoleLog("interrupted=%b", interrupted);

        elevatedShooter.shooter.stopShooting();
        robotDrive.disableTracking();
    }

    /**
     * Returns the degree and speed offsets for aiming based on the robot velocity.
     * This is because if we shoot while moving the Note keeps the inertia/momentum
     * so we need to rotate off target to make it go in.
     * @param theta the desired pitch of the shooter
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

        if (RobotBase.isReal())
            alpha *= -1;

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
