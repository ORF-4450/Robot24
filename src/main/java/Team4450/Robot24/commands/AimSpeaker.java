package Team4450.Robot24.commands;

import static Team4450.Robot24.Constants.DRIVE_DEADBAND;
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

    private final PIDController rotationController = new PIDController(0.01, 0, 0);

    private boolean initialMoveDone = false;

    public AimSpeaker(DriveBase robotDrive, ElevatedShooter elevatedShooter, PhotonVision shooterCamera, PhotonVision frontCamera, DoubleSupplier joystick) {
        this.robotDrive = robotDrive;
        this.elevatedShooter = elevatedShooter;
        this.shooterCamera = shooterCamera;
        this.frontCamera = shooterCamera;//frontCamera;
        this.joystick = joystick;

        rotationController.setTolerance(0.3);
        SendableRegistry.addLW(rotationController, "AimSpeaker Rotation PID");
    }

    @Override
    public void initialize() {
        Util.consoleLog();
        robotDrive.enableTracking();
    }

    @Override
    public void execute() {
        System.out.println(shooterCamera.getPitch());
        double currentAngle = elevatedShooter.shooter.getAngle();
        if (RobotBase.isSimulation()) shooterCamera.adjustSimCameraAngle(0, Math.toRadians(currentAngle), Math.toRadians(180));

        if (!initialMoveDone) {
            initialMoveDone = elevatedShooter.executeSetPosition(PresetPosition.SHOOT_VISION_START);
        }

        // rotation of robot ==============================
        PhotonTrackedTarget target = frontCamera.getTarget(tagNames.SPEAKER_MAIN);
        System.out.println(target);
        if (target == null) target = frontCamera.getTarget(tagNames.SPEAKER_OFFSET);
        double joystickValue = joystick.getAsDouble();
        if (joystickValue < -DRIVE_DEADBAND || joystickValue > DRIVE_DEADBAND) {
            robotDrive.setTrackingRotation(Double.NaN);
        } else {
            if (target == null) {
                robotDrive.setTrackingRotation(Double.NaN);
            } else {
                double output = rotationController.calculate(target.getYaw(), 0);
                robotDrive.setTrackingRotation(output);
            }
        }

        // shooter pivot ==================================
        target = shooterCamera.getTarget(tagNames.SPEAKER_MAIN);
        System.out.println(target);
        if (target == null) target = shooterCamera.getTarget(tagNames.SPEAKER_OFFSET);
        if (target != null) {
            // Util.consoleLog("%f %f", currentAngle, target.getPitch());
            double newAngle = currentAngle - target.getPitch();
            newAngle = Util.clampValue(newAngle, -60, 0);
            elevatedShooter.shooter.setAngle(newAngle);
        } else {
            // initialMoveDone = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        robotDrive.disableTracking();
    }

    @Override
    public boolean isFinished() {return false;}
}
