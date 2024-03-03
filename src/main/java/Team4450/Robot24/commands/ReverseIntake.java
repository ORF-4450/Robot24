package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class ReverseIntake extends Command {
    private final Intake  intake;
    private final DriveBase driveBase;

    public ReverseIntake(Intake intake, DriveBase driveBase) {
        this.intake = intake;
        this.driveBase = driveBase;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        // intake.start((driveBase.getChassisSpeeds().vxMetersPerSecond) / 8.0);
        intake.start(-1);
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        intake.stop();
    }
}
