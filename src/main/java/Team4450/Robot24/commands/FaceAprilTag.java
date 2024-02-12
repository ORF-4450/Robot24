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
        // get an arralist of all the tags IDs that the camera currently sees
        ArrayList<Integer> tags = photonVision.getTrackedIDs();
        PhotonTrackedTarget target;

        // first prioritize the center speaker tag if it is in view
        if (tags.contains(tagNames.SPEAKER_MAIN)) target = photonVision.getTarget(tagNames.SPEAKER_MAIN);

        // next try finding the amp tag, then L source, etc...
        else if (tags.contains(tagNames.AMP))target = photonVision.getTarget(tagNames.AMP);
        else if (tags.contains(tagNames.SOURCE_LEFT)) target = photonVision.getTarget(tagNames.SOURCE_LEFT);
        else if (tags.contains(tagNames.SOURCE_RIGHT)) target = photonVision.getTarget(tagNames.SOURCE_RIGHT);
        else if (tags.contains(tagNames.SPEAKER_OFFSET)) target = photonVision.getTarget(tagNames.SPEAKER_OFFSET);
        else if (tags.contains(tagNames.TRAP_BACK)) target = photonVision.getTarget(tagNames.TRAP_BACK);
        else if (tags.contains(tagNames.TRAP_LEFT)) target = photonVision.getTarget(tagNames.TRAP_LEFT);
        else if (tags.contains(tagNames.TRAP_RIGHT)) target = photonVision.getTarget(tagNames.TRAP_RIGHT);

        // finally, default to the first tag that is another alliance
        // could be offset speaker, trap source, etc, other alliance, etc.
        else if (tags.size() > 0) target = photonVision.getTarget(tags.get(0));

        // otherwise tell drivebase to set NaN as rotation to let driver override commanded
        // rotation to reorient the robot manually
        else {
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
