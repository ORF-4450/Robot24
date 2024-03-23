package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.ElevatedShooter.PresetPosition;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Move the shooter and elevator to one of many "PresetPosition"s defined in the
 * ElevatedShooter subsystem class. Very helpful to chain commands with in RobotContainer!
 */
public class Preset extends Command {
    private final ElevatedShooter elevatedShooter;
    private PresetPosition preset;

    private boolean done = false; // whether it's done or not

    /**
     * Move the shooter and elevator to one of many "PresetPosition"s defined in the
     * ElevatedShooter subsystem class.
     * @param elevatedShooter the ElevatedShooter subsystem
     * @param preset the PresetPosition to go to
     */
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
        // must be called every loop!
        // most of the code for this is in ElevatedShooter, so better to look
        // there for info/code
        done = elevatedShooter.executeSetPosition(preset);
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
    }
}
