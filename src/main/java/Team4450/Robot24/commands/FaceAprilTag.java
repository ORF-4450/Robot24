package Team4450.Robot24.commands;

import static Team4450.Robot24.Constants.alliance;

import java.util.ArrayList;

import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import Team4450.Robot24.utility.AprilTagNames;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.PhotonVision;

public class FaceAprilTag extends Command {
    DriveBase       robotDrive;
    PhotonVision    photonVision;
    PIDController   pidController = new PIDController(0.01, 0, 0);
    AprilTagNames   tagNames;

    public FaceAprilTag(DriveBase robotDrive, PhotonVision cameraSubsystem) {
        Util.consoleLog();

        tagNames = new AprilTagNames(alliance);

        // tolerance is in degrees.
        pidController.setTolerance(0.3);

        this.robotDrive = robotDrive;
        this.photonVision = cameraSubsystem;

        SendableRegistry.addLW(pidController, "AprilTag Rotate PID");
    }

    @Override
    public void initialize() {
        Util.consoleLog();

        pidController.reset();
        
        robotDrive.enableTracking();
    }
    
    @Override
    public void execute() {
        // get an arraylist of all the tags IDs that the camera currently sees
        ArrayList<Integer> tags = photonVision.getTrackedIDs();
        PhotonTrackedTarget target = null;

        // prioritize tags in this order:

        int[] tagPriorities = {
            tagNames.SPEAKER_MAIN,
            tagNames.AMP,
            tagNames.SOURCE_LEFT,
            tagNames.SOURCE_RIGHT,
            tagNames.SPEAKER_OFFSET,
            tagNames.TRAP_BACK,
            tagNames.TRAP_LEFT,
            tagNames.TRAP_RIGHT,

            tagNames.OPP_SPEAKER_MAIN,
            tagNames.OPP_AMP,
            tagNames.OPP_SOURCE_LEFT,
            tagNames.OPP_SOURCE_RIGHT,
            tagNames.OPP_SPEAKER_OFFSET,
            tagNames.OPP_TRAP_BACK,
            tagNames.OPP_TRAP_LEFT,
            tagNames.OPP_TRAP_RIGHT,
        };

        // loop through priorities and choose first one that it can see
        for (int i=0;i<tagPriorities.length;i++) {
            int tagToCheck = tagPriorities[i];
            if (tags.contains(tagToCheck)) { // if tag at priority i is in view
                target = photonVision.getTarget(tagToCheck); // set that as the target
                break; // and skip the rest
            }
        }

        // if no tags tell drivebase to set NaN as rotation to let driver override commanded
        // rotation to reorient the robot manually
        if (target == null) {
            robotDrive.setTrackingRotation(Double.NaN);
            SmartDashboard.putBoolean("Has AprilTag", false);
            return;
        }

        // attempt to use PID controller to make target yaw approach 0 degrees
        double output = pidController.calculate(target.getYaw(), 0);

        // override joystick rotation input and use the PID output to turn
        // the robot instead
        robotDrive.setTrackingRotation(output);

        SmartDashboard.putNumber("AprilTag ID", target.getFiducialId());
        SmartDashboard.putBoolean("Has AprilTag", true);
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        
        robotDrive.disableTracking();

        SmartDashboard.putBoolean("Has AprilTag", false);
        SmartDashboard.putNumber("AprilTag ID", 0);
    }
}
