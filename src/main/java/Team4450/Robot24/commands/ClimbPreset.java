package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.Elevator;
import Team4450.Robot24.subsystems.Intake;
import Team4450.Robot24.subsystems.Shooter;
import Team4450.Robot24.subsystems.ElevatedShooter.PresetPosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbPreset extends Command {
    private final ElevatedShooter elevatedShooter;

    private boolean done = false;

    public ClimbPreset(ElevatedShooter elevatedShooter) {
        this.elevatedShooter = elevatedShooter;
        addRequirements(elevatedShooter);
    }

    @Override
    public void execute() {
        done = elevatedShooter.executeSetPosition(PresetPosition.CLIMB);
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        elevatedShooter.elevator.stopMoving();
        Util.consoleLog("interrupted=%b", interrupted);
    }
}
