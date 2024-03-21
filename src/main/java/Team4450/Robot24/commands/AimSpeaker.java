package Team4450.Robot24.commands;

import static Team4450.Robot24.Constants.PODIUM_ANGLE;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AimSpeaker extends Command {
    private final DriveBase robotDrive;
    private final ElevatedShooter elevatedShooter;
    private final PhotonVision shooterCamera;
    private final PhotonVision frontCamera;
    private final DoubleSupplier joystick;
    private boolean upperShot = false;
    private final AprilTagNames tagNames = new AprilTagNames(alliance);

    private final PIDController rotationController = new PIDController(0.02, 0, 0);
    private final PIDController pivotController = new PIDController(0.06, 0, 0);

    private final InterpolatingDoubleTreeMap pitchOffsets = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap yawOffsets = new InterpolatingDoubleTreeMap();

    public AimSpeaker(DriveBase robotDrive, ElevatedShooter elevatedShooter, PhotonVision shooterCamera, DoubleSupplier joystick, boolean upperShot) {
        this.robotDrive = robotDrive;
        this.elevatedShooter = elevatedShooter;
        this.shooterCamera = shooterCamera;
        this.frontCamera = shooterCamera;//frontCamera;
        this.upperShot = upperShot;
        this.joystick = joystick;

        addRequirements(elevatedShooter);

        rotationController.setTolerance(0.3);
        SendableRegistry.addLW(rotationController, "AimSpeaker Rotation PID");
        SendableRegistry.addLW(pivotController, "Pivot Rotation PID");

        // mapping of distance (meters) to pitch offset
        if (!upperShot) {
            pitchOffsets.put(1.45, -50.0);
            pitchOffsets.put(2.37, -39.0);
            pitchOffsets.put(3.45, -26.0);
            pitchOffsets.put(3.69, -22.0);
        } else {
            pitchOffsets.put(1.45, -44.0);
            pitchOffsets.put(2.37, -33.0);
            pitchOffsets.put(3.45, -20.0);
            pitchOffsets.put(3.69, -16.0);
        }
        
        

        // mapping of distance (meters) to yaw value
        yawOffsets.put(0.0, 6.0);
        yawOffsets.put(20.0, 6.0);

        SmartDashboard.putBoolean("Target Locked", false);
    }

    public AimSpeaker(DriveBase robotDrive, ElevatedShooter elevatedShooter, PhotonVision shooterCamera, DoubleSupplier joystick) {
        this(robotDrive, elevatedShooter, shooterCamera, joystick, false);
    }    

    @Override
    public void initialize() {
        Util.consoleLog();
        SmartDashboard.putBoolean("Target Locked", false);
        robotDrive.enableTracking();
    }

    @Override
    public void execute() {
        elevatedShooter.shooter.startShooting();
        double currentAngle = elevatedShooter.shooter.getAngle();
        // if (RobotBase.isSimulation()) shooterCamera.adjustSimCameraAngle(0, Math.toRadians(currentAngle), Math.toRadians(180));

        int targetId = tagNames.SPEAKER_MAIN;
        PhotonTrackedTarget target = frontCamera.getTarget(targetId);


        if (target == null ) {
            Util.consoleLog("null target");
            SmartDashboard.putBoolean("Target Locked", false);
            SmartDashboard.putNumber("Distance to Speaker", Double.NaN);
            robotDrive.setTrackingRotation(Double.NaN);
            return;
        }

        boolean yawOkay = false;
        // boolean pitchOkay = false;
        double dist = PhotonUtils.calculateDistanceToTargetMeters(0.57, 1.4224, // camera height, tag height (center)
            Math.toRadians(25),
            Math.toRadians(target.getPitch())
        );

        // rotation of robot ==============================

        double joystickValue = joystick.getAsDouble();
        if (joystickValue < -0.03 || joystickValue > 0.03) {robotDrive.setTrackingRotation(Double.NaN);}
        else {
            double output = rotationController.calculate(target.getYaw(), yawOffsets.get(dist));
            if (Math.abs(output) < 0.02) yawOkay = true;
            robotDrive.setTrackingRotation(output);
        }


        // shooter pivot ==================================
        // double offset = target.getPitch() - pitchOffsets.get(dist);
        // double newAngle = Util.clampValue(currentAngle - offset, -90, 0); // for safety
        elevatedShooter.shooter.setAngle(pitchOffsets.get(dist));

        Util.consoleLog("dist=%f angle=%f", dist, pitchOffsets.get(dist));
        Util.consoleLog("shooter_angle=%f pitch=%f yaw=%f", currentAngle, target.getPitch(), target.getYaw());
        // if (Math.abs(offset) < 1) pitchOkay = true;
        SmartDashboard.putBoolean("Target Locked", yawOkay);
        SmartDashboard.putNumber("Distance to Speaker", dist);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Target Locked", false);
        Util.consoleLog("interrupted=%b", interrupted);
        // if (interrupted)
            elevatedShooter.shooter.stopShooting();
        robotDrive.disableTracking();
    }

    @Override
    public boolean isFinished() {
        return elevatedShooter.shooter.hasShot;
    }
}
