package Team4450.Robot24.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.PhotonVision;

public class DriveToNote extends Command {
    PIDController rotationController = new PIDController(0.02, 0, 0); // for rotating drivebase
    PIDController translationController = new PIDController(0.02, 0, 0); // for moving drivebase in X,Y plane
    DriveBase robotDrive;
    PhotonVision photonVision;
    private boolean alsoDrive;
    private boolean initialFieldRel;

    /**
     * Track to a note using getArea() and getYaw()
     * @param robotDrive the robot drive base
     * @param photonVision the photonvision subsystem
     */
    public DriveToNote(DriveBase robotDrive, PhotonVision photonVision, boolean alsoDrive) {
        this.robotDrive = robotDrive;
        this.photonVision = photonVision;
        this.alsoDrive = alsoDrive;

        SendableRegistry.addLW(translationController, "DriveToNote Translation PID");
        SendableRegistry.addLW(rotationController, "DriveToNote Rotation PID");
    }

    @Override
    public void initialize() {
        Util.consoleLog();

        initialFieldRel = robotDrive.getFieldRelative();
        if (initialFieldRel)
            robotDrive.toggleFieldRelative();

        robotDrive.enableTracking();

        rotationController.setSetpoint(0); // target should be at yaw=0 degrees
        rotationController.setTolerance(1.5); // withing 0.5 degrees of 0

        translationController.setSetpoint(0); // target should be at -15 pitch
        translationController.setTolerance(0.5);
    }

    @Override
    public void execute() {
        if (!photonVision.hasTargets()) {
            robotDrive.setTrackingRotation(Double.NaN);
            return;
        }

        PhotonTrackedTarget target = photonVision.getClosestTarget();

        double rotation = rotationController.calculate(target.getYaw());
        double movement = translationController.calculate(target.getPitch());

        Util.consoleLog("in[yaw=%f, pitch=%f] out[rot=%f, mov=%f]", target.getYaw(), target.getPitch(), rotation, movement);

        if (alsoDrive) {
            robotDrive.driveRobotRelative(-movement, 0, rotation);
        } else {
            robotDrive.setTrackingRotation(rotation);
        }
    }
    

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        if (initialFieldRel)
            robotDrive.toggleFieldRelative(); // toggle back to beginning state
        robotDrive.setTrackingRotation(Double.NaN);
        robotDrive.disableTracking();
    }
}