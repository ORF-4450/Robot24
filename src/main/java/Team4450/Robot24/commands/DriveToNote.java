package Team4450.Robot24.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.PhotonVision;

/**
 * Faces the "closest" note and optionally drives towards it, also enables robot oriented controls
 */
public class DriveToNote extends Command {
    PIDController rotationController = new PIDController(0.008, 0, 0); // for rotating drivebase
    PIDController translationController = new PIDController(0.02, 0, 0); // for moving drivebase in X,Y plane
    DriveBase robotDrive;
    PhotonVision photonVision;
    private boolean alsoDrive;
    private boolean initialFieldRel;

    /**
     * Track to a note using getPitch() and getYaw(), while turning on
     * robot relative controls for ease of driving
     * @param robotDrive the drivebase
     * @param photonVision the camera
     * @param alsoDrive whether to drive to the note or just face it
     */
    public DriveToNote(DriveBase robotDrive, PhotonVision photonVision, boolean alsoDrive) {
        this.robotDrive = robotDrive;
        this.photonVision = photonVision;
        this.alsoDrive = alsoDrive;

        if (alsoDrive)
            addRequirements(robotDrive); // require if also drive. TODO: may need to change for auto

        SendableRegistry.addLW(translationController, "DriveToNote Translation PID");
        SendableRegistry.addLW(rotationController, "DriveToNote Rotation PID");
    }

    @Override
    public void initialize() {
        Util.consoleLog();

        // store the initial field relative state to reset it later.
        initialFieldRel = robotDrive.getFieldRelative();
        if (initialFieldRel)
            robotDrive.toggleFieldRelative(); // turn field relative off (to robot oriented) if it's on

        robotDrive.enableTracking(); // allow rotation control

        rotationController.setSetpoint(0); // target should be at yaw=0 degrees
        rotationController.setTolerance(1.5); // withing 1.5 degrees of 0

        translationController.setSetpoint(-15); // target should be at -15 pitch
        translationController.setTolerance(0.5);
    }

    @Override
    public void execute() {
        // logic for chosing "closest" target in PV subsystem
        PhotonTrackedTarget target = photonVision.getClosestTarget();

        // better to check if target == null than using hasTargets() because tgere is a rare
        // race condition where the target disappears between getTarget and hasTargets which is crazy
        // but it happened once...
        if (target == null) {
            robotDrive.setTrackingRotation(Double.NaN); // temporarily disable tracking
            return;
        }

        double rotation = rotationController.calculate(target.getYaw()); // attempt to minimize
        double movement = translationController.calculate(target.getPitch()); // attempt to minimize

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
            robotDrive.toggleFieldRelative(); // restore beginning state
        robotDrive.setTrackingRotation(Double.NaN);
        robotDrive.disableTracking();
    }
}