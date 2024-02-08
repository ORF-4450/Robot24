package Team4450.Robot24.commands;

import Team4450.Robot24.subsystems.Intake;
import Team4450.Robot24.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootNote extends Command {
    private final Shooter shooter;

    public ShootNote(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.startShooting();
    }

    @Override
    public void execute() {
        shooter.startFeeding();
    }
}
