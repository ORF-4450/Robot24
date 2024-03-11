package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class ReverseIntake extends Command {
    private final Intake  intake;
    private final DriveBase driveBase;
    private final ElevatedShooter elevatedShooter;

    public ReverseIntake(Intake intake, ElevatedShooter elevatedShooter, DriveBase driveBase) {
        this.intake = intake;
        this.elevatedShooter = elevatedShooter;
        this.driveBase = driveBase;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        // intake.start((driveBase.getChassisSpeeds().vxMetersPerSecond) / 8.0);
        elevatedShooter.shooter.startFeeding(-1);
        intake.start(-1);
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        elevatedShooter.shooter.stopFeeding();
        intake.stop();
    }
}
