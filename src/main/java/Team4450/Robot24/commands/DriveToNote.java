package Team4450.Robot24.commands;

import Team4450.Lib.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.PhotonVision;

public class DriveToNote extends Command {
    PIDController rotationController = new PIDController(0.01, 0, 0); // for rotating drivebase
    PIDController translationController = new PIDController(0.1, 0, 0); // for moving drivebase in X,Y plane
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

        SendableRegistry.addLW(translationController, "TrackNote Translation PID");
        SendableRegistry.addLW(rotationController, "TrackNote Rotation PID");
        //SmartDashboard.putData("trackNoteTranslationPID", translationController);
        //SmartDashboard.putData("trackNoteRotationPID", rotationController);
    }

    @Override
    public void initialize() {
        Util.consoleLog();

        // photonVision.selectPipeline(0); // TODO: set pipeline

        rotationController.setSetpoint(0); // target should be at yaw=0 degrees
        rotationController.setTolerance(0.5); // withing 0.5 degrees of 0

        translationController.setSetpoint(90); // target should fill 90% of total camera FOV
    }

    @Override
    public void execute() {
        if (!photonVision.hasTargets()) return;
        // make sure target centered before we move
        if (!rotationController.atSetpoint()) {
            double rotation = rotationController.calculate(photonVision.getYaw());
            robotDrive.driveRobotRelative(0, 0, rotation);
        }
        // otherwise drive to the target (only forwards backwards)
        else {
            double movement = translationController.calculate(photonVision.getArea());
            robotDrive.driveRobotRelative(-movement, 0, 0); // negative because camera backwards.
        }
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
    }
}