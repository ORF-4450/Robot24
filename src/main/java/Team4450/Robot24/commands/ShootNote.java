package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootNote extends Command {
    private final Shooter shooter;
    private double startTime;
    private boolean hasNote = true; //TODO: change this

    public ShootNote(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    @Override
    public void initialize() {
        shooter.startShooting();
        shooter.startFeeding(1);
        startTime = Util.timeStamp();
    }

    @Override
    public void execute() {
        if (Util.getElaspedTime(startTime) > 0.5) {
            shooter.stopFeeding();
            shooter.stopShooting();
        }
    }

    @Override
    public boolean isFinished() {
        boolean timeHasElasped = Util.getElaspedTime(startTime) > 0.55;
        return timeHasElasped || !hasNote;
    }
}
