package Team4450.Robot24.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.PhotonVision;

public class DriveToNote extends Command {
    PIDController rotationController = new PIDController(0.05, 0.5, 0); // for rotating drivebase
    PIDController translationController = new PIDController(0.3, 0, 0); // for moving drivebase in X,Y plane
    DriveBase robotDrive;
    PhotonVision photonVision;

    /**
     * Track to a note using getArea() and getYaw()
     * @param robotDrive the robot drive base
     * @param photonVision the photonvision subsystem
     */
    public DriveToNote(DriveBase robotDrive, PhotonVision photonVision) {
        this.robotDrive = robotDrive;
        this.photonVision = photonVision;

        SendableRegistry.addLW(translationController, "DriveToNote Translation PID");
        SendableRegistry.addLW(rotationController, "DriveToNote Rotation PID");
    }

    @Override
    public void initialize() {
        Util.consoleLog();

        rotationController.setSetpoint(0); // target should be at yaw=0 degrees
        rotationController.setTolerance(0.5); // withing 0.5 degrees of 0

        translationController.setSetpoint(-15); // target should be at -15 pitch
        translationController.setTolerance(0.5);
    }

    @Override
    public void execute() {
        if (!photonVision.hasTargets()) return;

        PhotonTrackedTarget target = photonVision.getClosestTarget();

        // Util.consoleLog("yaw=%f", target.getYaw());
        // Util.consoleLog("pitch=%f", target.getPitch());

        double rotation = rotationController.calculate(target.getYaw());
        double movement = translationController.calculate(target.getPitch());

        // make sure target centered before we move
        if (!rotationController.atSetpoint()) {
            robotDrive.driveRobotRelative(0, 0, rotation);
        }
        // otherwise drive to the target (only forwards backwards)
        else if (!translationController.atSetpoint()) {
            robotDrive.driveRobotRelative(-movement, 0, 0); // negative because camera backwards.
        }
    }
    

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
    }
}