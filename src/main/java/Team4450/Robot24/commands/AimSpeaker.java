package Team4450.Robot24.commands;

import static Team4450.Robot24.Constants.DRIVE_DEADBAND;
import static Team4450.Robot24.Constants.PV_TARGET_PITCH;
import static Team4450.Robot24.Constants.alliance;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonVersion;
import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;
import Team4450.Robot24.AdvantageScope;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.Elevator;
import Team4450.Robot24.subsystems.PhotonVision;
import Team4450.Robot24.subsystems.Shooter;
import Team4450.Robot24.utility.AprilTagNames;
import Team4450.Robot24.subsystems.ElevatedShooter.PresetPosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AimSpeaker extends Command {
    private final DriveBase robotDrive;
    private final ElevatedShooter elevatedShooter;
    private final PhotonVision shooterCamera;
    private final PhotonVision frontCamera;
    private final DoubleSupplier joystick;
    private final AprilTagNames tagNames = new AprilTagNames(alliance);

    private final PIDController rotationController = new PIDController(0.02, 0, 0);
    private final PIDController pivotController = new PIDController(0.06, 0, 0);

    private final InterpolatingDoubleTreeMap pitchOffsets = new InterpolatingDoubleTreeMap();

    private boolean initialMoveDone = false;

    public AimSpeaker(DriveBase robotDrive, ElevatedShooter elevatedShooter, PhotonVision shooterCamera, DoubleSupplier joystick) {
        this.robotDrive = robotDrive;
        this.elevatedShooter = elevatedShooter;
        this.shooterCamera = shooterCamera;
        this.frontCamera = shooterCamera;//frontCamera;
        this.joystick = joystick;

        addRequirements(elevatedShooter);

        rotationController.setTolerance(0.3);
        SendableRegistry.addLW(rotationController, "AimSpeaker Rotation PID");
        SendableRegistry.addLW(pivotController, "Pivot Rotation PID");

        pitchOffsets.put(0.13, 5.5);
        // pitchOffsets.put(0.5, -19.5);
        pitchOffsets.put(0.85, 6.5);

    }

    @Override
    public void initialize() {
        Util.consoleLog();
        SmartDashboard.putBoolean("Target Locked", false);
        robotDrive.enableTracking();
        initialMoveDone = false;
    }

    @Override
    public void execute() {
        elevatedShooter.shooter.startShooting();
        double currentAngle = elevatedShooter.shooter.getAngle();
        if (RobotBase.isSimulation()) shooterCamera.adjustSimCameraAngle(0, Math.toRadians(currentAngle), Math.toRadians(180));

        if (!initialMoveDone) {
            initialMoveDone = elevatedShooter.executeSetPosition(PresetPosition.SHOOT_VISION_START);
            return;
        }

        boolean yawOkay = false;
        boolean pitchOkay = false;

        // rotation of robot ==============================
        PhotonTrackedTarget target = frontCamera.getTarget(tagNames.SPEAKER_MAIN);
        double joystickValue = joystick.getAsDouble();
        if (joystickValue < -0.01 || joystickValue > 0.01) {
            robotDrive.setTrackingRotation(Double.NaN);
        } else {
            if (target == null) {
                robotDrive.setTrackingRotation(Double.NaN);
            } else {
                double output = rotationController.calculate(target.getYaw(), 7);
                if (Math.abs(output) < 0.01) yawOkay = true;
                robotDrive.setTrackingRotation(output);
            }
        }

        // shooter pivot ==================================
        // if (target == null) target = shooterCamera.getTarget(tagNames.SPEAKER_OFFSET);
        if (target != null) {
            // double power = pivotController.calculate(target.getPitch(), -29);
            double offset = target.getPitch() - pitchOffsets.get(target.getArea());
            Util.consoleLog("area=%f pitch_offset=%f", target.getArea(), pitchOffsets.get(target.getArea()));
            double newAngle = currentAngle - offset;
            if (Math.abs(offset) < 1) pitchOkay = true;
            newAngle = Util.clampValue(newAngle, -90, 0);
                Util.consoleLog("angle=%f pitch=%f newangle=%f yaw=%f", currentAngle, target.getPitch(), newAngle, target.getYaw());
            elevatedShooter.shooter.setAngle(newAngle);
            // elevatedShooter.shooter.movePivotRelative(power);
        } else {
            // initialMoveDone = false;
        }

        SmartDashboard.putBoolean("Target Locked", yawOkay && pitchOkay);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Target Locked", false);
        Util.consoleLog("interrupted=%b", interrupted);
        robotDrive.disableTracking();
    }

    @Override
    public boolean isFinished() {return false;}
}
