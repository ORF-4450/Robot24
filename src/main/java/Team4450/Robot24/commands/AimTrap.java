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

/**
 * Positions robot to face directly at a trap, given a specific X and Y offset from the tag,
 * while also spinning up the shooter wheels at a special low power for trap shooting.
 */
public class AimTrap extends Command {
    private PIDController rotationController = new PIDController(0.008, 0, 0); // for rotating drivebase
    private PIDController xTranslationController = new PIDController(0.3, 0, 0); // for moving drivebase in X,Y plane
    private PIDController yTranslationController = new PIDController(0.3, 0, 0); // for moving drivebase in X,Y plane

    // april tag name helper
    private AprilTagNames tagNames = new AprilTagNames(alliance);

    private DriveBase robotDrive;
    private PhotonVision photonVision;

    /**
     * Positions robot to face directly at a trap, given a specific X and Y offset from the tag,
     * while also spinning up the shooter wheels at a special low power for trap shooting.
     * @param robotDrive the drivebase
     * @param photonVision the camera instance to use
     */
    public AimTrap(DriveBase robotDrive, PhotonVision photonVision) {
        this.robotDrive = robotDrive;
        this.photonVision = photonVision;

        addRequirements(robotDrive); // we override all movement so no reason to not require

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

        // prioritize the left, then right, then back trap.
        target = photonVision.getTarget(tagNames.TRAP_LEFT);
        if (target == null) target = photonVision.getTarget(tagNames.TRAP_RIGHT);
        if (target == null) target = photonVision.getTarget(tagNames.TRAP_BACK);
        if (target == null) {
            Util.consoleLog("null trap target");
            return;
        }

        Util.consoleLog("trap found %d", target.getFiducialId());
        
        // calculate the camera-to-tag transformation, this gets a little
        // bit complicated but I'll do my best to explain - cole
        Transform3d transform = target.getBestCameraToTarget();

        // get the Z-axis of rotation (yaw) from the transformation. This is essentially 180 deg.
        // if the camera was somehow facing the exact same direction as the tag (physically impossible),
        // then this would be 0. However, it's never quite 180 deg because output is bounded [-180, 180].
        // so it jumps from -179.999 to 179.999. The if statements below normalize this to reverse it so that 0 is 180,
        // and 180 is 0, etc.
        // essentially after all this when yaw is 0 then the planes of tag and camera front are parallel on the Z axis.
        double yaw = Math.toDegrees(transform.getRotation().getZ());
        if (yaw > 0) {yaw = 180 - yaw;}
        else {yaw = - (yaw + 180);}

        // X and Y are switched on purpose: the "x" direction is downfield for robot
        // but sideways for tag. we convert tag relative to robot relative for simplicity
        double xOffset = transform.getY();
        double yOffset = transform.getX();

        Util.consoleLog("yaw=%f, x=%f, y=%f", yaw, xOffset, yOffset);

        double rotOutput = -rotationController.calculate(yaw, 0); // not sure why negative but keep it that way
        double xOutput = xTranslationController.calculate(xOffset, 0); // 0 meters offset from side to side
        double yOutput = yTranslationController.calculate(yOffset, 1.2); // 1.2 meters offset from front

        robotDrive.driveRobotRelative(yOutput, xOutput, rotOutput); // use outputs
    }
    

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        robotDrive.driveRobotRelative(0, 0, 0); // stop moving
    }
}