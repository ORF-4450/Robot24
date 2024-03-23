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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableRegistry;
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
            pitchOffsets.put(1.45, -50.0);
            pitchOffsets.put(2.37, -39.0);
            pitchOffsets.put(3.45, -26.0);
            pitchOffsets.put(3.69, -22.0);
        }
        
    
        // mapping of distance (meters) to target yaw offset value within frame
        // Ideally, the yaw offset should be 0 deg because that is
        // perfectly centered within PV frame, but that's only if the camera is on
        // the center of the robot. If it is on the side we have to "center" a little
        // bit off to the side:
        yawOffsets.put(0.0, 6.0);
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
        elevatedShooter.shooter.startShooting(); // keep the wheels spun up!

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

        // ============= rotation of robot ==============================
        // points the physical drivebase at the speaker usig yaw offsets

        double joystickValue = joystick.getAsDouble(); // check if joystick is moving
        if (joystickValue < -0.03 || joystickValue > 0.03) {robotDrive.setTrackingRotation(Double.NaN);} // if so, just do joystick
        else {
            // if joystick not moving, use PID to attempt to match the yaw offset interpolated using above values
            // at the current distance
            double output = rotationController.calculate(target.getYaw(), yawOffsets.get(dist));
            if (Math.abs(output) < 0.02) yawOkay = true; // if within tolerance than the yaw is good
            robotDrive.setTrackingRotation(output); // will be handled in drivebase as "faked" joystick input
        }


        // ============ shooter pivot ==================================
        // pivots the shooter to the interpolated angle
        //
        // NOTE: at intake position it's not safe to pivot,
        // another command must first raise elevator a little
        double setpoint = pitchOffsets.get(dist); // degrees
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

    @Override
    public boolean isFinished() {
        // this will be set by another command
        return elevatedShooter.shooter.hasShot;
    }
}
