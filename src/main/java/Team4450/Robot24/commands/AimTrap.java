package Team4450.Robot24.commands;

import static Team4450.Robot24.Constants.alliance;

import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.PhotonVision;
import Team4450.Robot24.utility.AprilTagNames;

public class AimTrap extends Command {
    PIDController rotationController = new PIDController(0.008, 0, 0); // for rotating drivebase
    PIDController xTranslationController = new PIDController(0.3, 0, 0); // for moving drivebase in X,Y plane
    PIDController yTranslationController = new PIDController(0.3, 0, 0); // for moving drivebase in X,Y plane

    AprilTagNames tagNames = new AprilTagNames(alliance);

    DriveBase robotDrive;
    PhotonVision photonVision;
    ElevatedShooter elevatedShooter;

    /**
     * Track to a note using getArea() and getYaw()
     * @param robotDrive the robot drive base
     * @param photonVision the photonvision subsystem
     */
    public AimTrap(DriveBase robotDrive, PhotonVision photonVision, ElevatedShooter elevatedShooter) {
        this.robotDrive = robotDrive;
        this.photonVision = photonVision;
        this.elevatedShooter = elevatedShooter;

        SendableRegistry.addLW(yTranslationController, "Trap Translation PID");
        SendableRegistry.addLW(rotationController, "Trap Rotation PID");
    }

    @Override
    public void initialize() {
        Util.consoleLog();
    }

    @Override
    public void execute() {
        PhotonTrackedTarget target;

        target = photonVision.getTarget(tagNames.TRAP_LEFT);
        if (target == null) target = photonVision.getTarget(tagNames.TRAP_RIGHT);
        if (target == null) target = photonVision.getTarget(tagNames.TRAP_BACK);
        if (target == null) {
            Util.consoleLog("null trap target");
            return;
        }

        Util.consoleLog("trap found %d", target.getFiducialId());
        
        Transform3d transform = target.getBestCameraToTarget();

        double yaw = Math.toDegrees(transform.getRotation().getZ());
        if (yaw > 0) {
            yaw = 180 - yaw;
        } else {
            yaw = - (yaw + 180);
        }

        double xOffset = transform.getY();
        // if (yaw > 0)
        //     xOffset *= -1;
        double yOffset = transform.getX(); // this is supposed to be X not Y even though it looks wrong

        Util.consoleLog("yaw=%f, x=%f, y=%f", yaw, xOffset, yOffset);

        double rotOutput = -rotationController.calculate(yaw, 0);
        double xOutput = xTranslationController.calculate(xOffset, 0);
        double yOutput = yTranslationController.calculate(yOffset, 1.2);

        robotDrive.driveRobotRelative(yOutput, xOutput, rotOutput);

        elevatedShooter.shooter.startShooting(0.3);
    }
    

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        robotDrive.driveRobotRelative(0, 0, 0);
        elevatedShooter.shooter.stopShooting();
    }
}