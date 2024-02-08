package Team4450.Robot24.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import Team4450.Robot24.subsystems.DriveBase;

public class PointToYaw extends Command {
    private DoubleSupplier  yawSupplier;
    private boolean         wait;
    private DriveBase       robotDrive;
    private PIDController   pidController = new PIDController(0.5, 0, 0);
    private Set<Subsystem>  requirements;

    private static final double NO_VALUE = Double.NaN;

    /**
     * Point to a yaw value
     * @param yawSupplier a supplier of desired yaw values (IN RADIANS!)
     * @param robotDrive the drive subsystem
     * @param wait whether or not to wait until it is completed to drive again (whether this command "requires" drivebase)
     */
    public PointToYaw(DoubleSupplier yawSupplier, DriveBase robotDrive, boolean wait) {
        Util.consoleLog();

        this.yawSupplier = yawSupplier;
        this.robotDrive = robotDrive;
        this.wait = wait;
        this.requirements = Set.of();

        SendableRegistry.addLW(pidController, "PointToYaw PID");

        // if wait is set to true, then "require" the drive subsystem to ovverride other commands
        if (wait) this.requirements = Set.of(robotDrive);
    }

    @Override
    public void execute() {
        // fetches desired yaw
        double desiredYaw = yawSupplier.getAsDouble();

        if (!Double.isNaN(desiredYaw)) // if NaN just finish up with last value
            pidController.setSetpoint(desiredYaw);

        if (Double.isNaN(desiredYaw) && !wait) {
            // if desiredYaw is NaN, that means that joystick is centered or POV
            // buttons aren't pressed or some other reason to temporarily ignore
            // the output of this command

            // if setTrackingRotation uses NaN, then the drivebase will ignore it and
            // just use joystick values
            robotDrive.setTrackingRotation(desiredYaw);
            return;
        }

        // calculate needed rotation with robot yaw (in radians) as input
        double rotation = -pidController.calculate(robotDrive.getYawR() % (Math.PI * 2));

        if (wait) {
            // if this command is only one running on drivebase (wait) then command it to run
            robotDrive.drive(0,0,rotation,false);
        }
        
        // sets the override in drivebase so it will use rotation rather than joystick
        robotDrive.setTrackingRotation(rotation);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        if (wait)
            return InterruptionBehavior.kCancelIncoming;
        else
            return InterruptionBehavior.kCancelSelf;
    }

    @Override
    public void initialize() {
        Util.consoleLog();

        pidController.reset();
        pidController.setTolerance(.05);      // in radians.
        pidController.enableContinuousInput(0, 2 * Math.PI); // rotation is continuous: full circle repeats
        robotDrive.enableTracking();
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("ended interrupted: %b", interrupted);

        robotDrive.disableTracking();
        robotDrive.setTrackingRotation(0);
    }

    @Override
    public boolean isFinished() {
        // end command when at setpoint if waiting otherwise just don't end
        if (wait)
            return pidController.atSetpoint();
        else
            return false;
    }

    /**
    * Generate a yaw from a POV value.
    *
    * @param pov The POV value.
    * @return yaw
    */
    public static double yawFromPOV(double pov) {
        if (pov < 0)
            // no POV buttons are pressed currently
            return NO_VALUE;
        else {
            // converts the 0 to 360 degree output of POV to yaw-style radian heading
            double radians = Math.toRadians(pov);

            if (radians > Math.PI) {
                double overshoot = radians - Math.PI;
                radians = -overshoot;
            }

            radians *= -1;
            return radians;
        }
    }
    
    /**
    * generate a yaw from axis values
    *
    * @param xAxis
    * @param yAxis
    * @return yaw
    */
    public static double yawFromAxes(double xAxis, double yAxis) {
        double theta = Math.atan2(xAxis, yAxis) + Math.PI;
        Util.consoleLog("%f", theta);
        double magnitude = Math.sqrt(Math.pow(xAxis, 2) + Math.pow(yAxis, 2));

        if (magnitude > 0.2) {
            return theta;
        } else {
            // this means the driver hasn't moved enough to trigger rotation
            // kind of like deadzone but radius of angle instead of individual
            // X and Y parts
            return NO_VALUE;
        }
    }
}