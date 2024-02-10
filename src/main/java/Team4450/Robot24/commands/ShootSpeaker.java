package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.Elevator;
import Team4450.Robot24.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootSpeaker extends Command {
    private final Shooter shooter;
    private final Elevator elevator;
    private double startTime;
    private boolean hasNote = true; //TODO: change this

    public ShootSpeaker(Shooter shooter, Elevator elevator) {
        this.shooter = shooter;
        this.elevator = elevator;
        addRequirements(shooter, elevator);
    }
    @Override
    public void initialize() {
        shooter.startFeeding(-0.3);
        startTime = Util.timeStamp();
    }

    @Override
    public void execute() {
        if (Util.getElaspedTime(startTime) > 0.2) {
            shooter.startShooting();
            shooter.startFeeding(1);
        }
        if (Util.getElaspedTime(startTime) > 1.2) {
            shooter.stopFeeding();
            shooter.stopShooting();
        }
    }

    @Override
    public boolean isFinished() {
        boolean timeHasElasped = Util.getElaspedTime(startTime) > 1.7;
        return timeHasElasped || !hasNote;
    }
}
