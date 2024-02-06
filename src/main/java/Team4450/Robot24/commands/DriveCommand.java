package Team4450.Robot24.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import static Team4450.Robot24.Constants.*;

import java.util.function.DoubleSupplier;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.DriveBase;

public class DriveCommand extends Command 
{
    private final DriveBase driveBase;

    private final DoubleSupplier throttleSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;
    private final XboxController controller;
    
    // private final SlewRateLimiter slewX = new SlewRateLimiter(Constants.DriveConstants.kMagnitudeSlewRate);
    // private final SlewRateLimiter slewY = new SlewRateLimiter(Constants.DriveConstants.kMagnitudeSlewRate);
    // private final SlewRateLimiter slewRot = new SlewRateLimiter(Constants.DriveConstants.kMagnitudeSlewRate);

    public DriveCommand(DriveBase driveBase,
                        DoubleSupplier throttleSupplier,
                        DoubleSupplier strafeSupplier,
                        DoubleSupplier rotationSupplier,
                        XboxController controller) 
    {
        Util.consoleLog();

        this.driveBase = driveBase;
        this.throttleSupplier = throttleSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.controller = controller;

        addRequirements(driveBase);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();
    }

    @Override
    public void execute() 
    {
        LCD.printLine(2, "rx=%.3f  ry=%.3f  throttle=%.3f  strafe=%.3f  rot=%.3f",
            controller.getRightX(),
            controller.getRightY(),
            throttleSupplier.getAsDouble(),
            strafeSupplier.getAsDouble(),
            rotationSupplier.getAsDouble()
        );

        LCD.printLine(3, "lx=%.3f  ly=%.3f  yaw=%.3f",
            controller.getLeftX(),
            controller.getLeftY(),
            //driveBase.getGyroRotation2d().getDegrees(),
            driveBase.getGyroYaw()
        );

        // This is the default command for the DriveBase. When running in autonmous, the auto commands
        // require DriveBase, which preempts the default DriveBase command. However, if our auto code ends 
        // before end of auto period, then this drive command resumes and is feeding drivebase during remainder
        // of auto period. This was not an issue until the joystick drift problems arose, so the resumption of a 
        // driving command during auto had the robot driving randomly after our auto program completed. The if 
        // statment below prevents this.
        
        if (robot.isAutonomous()) return;

        double throttle = deadband(throttleSupplier.getAsDouble(), DRIVE_DEADBAND);
        double strafe = deadband(strafeSupplier.getAsDouble(), DRIVE_DEADBAND);
        double rotation = deadband(rotationSupplier.getAsDouble(), ROTATION_DEADBAND);

        // Have to invert for sim...not sure why.
        if (RobotBase.isSimulation()) rotation *= -1;
        
        driveBase.drive(throttle, strafe, rotation, false);
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);

        //driveBase.drive(0.0, 0.0, 0.0);
    }
 
    private static double deadband(double value, double deadband) 
    {
        return Math.abs(value) > deadband ? value : 0.0;
    }

    private static double squareTheInput(double value) 
    {
        return Math.copySign(value * value, value);
    }
}
