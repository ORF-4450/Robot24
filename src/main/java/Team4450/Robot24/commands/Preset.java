package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.ElevatedShooter.PresetPosition;
import edu.wpi.first.wpilibj2.command.Command;

public class Preset extends Command {
    private final ElevatedShooter elevatedShooter;
    private PresetPosition preset;

    private boolean done = false;

    public Preset(ElevatedShooter elevatedShooter, PresetPosition preset) {
        this.elevatedShooter = elevatedShooter;
        this.preset = preset;
        addRequirements(elevatedShooter);
    }

    @Override
    public void initialize() {
        Util.consoleLog();
    }

    @Override
    public void execute() {
        done = elevatedShooter.executeSetPosition(preset);
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        // elevatedShooter.elevator.stopMoving();
        Util.consoleLog("interrupted=%b", interrupted);
    }
}
