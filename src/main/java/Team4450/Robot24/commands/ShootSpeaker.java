package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.Elevator;
import Team4450.Robot24.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootSpeaker extends Command {
    private final Shooter shooter;
    private final Elevator elevator;
    private double startTime;

    public ShootSpeaker(Shooter shooter, Elevator elevator) {
        this.shooter = shooter;
        this.elevator = elevator;
        addRequirements(shooter, elevator);
    }
    @Override
    public void initialize() {
        // start by feeding the note backwards a bit (0.2 seconds)
        shooter.startFeeding(-0.3);
        startTime = Util.timeStamp();
    }

    @Override
    public void execute() {
        if (Util.getElaspedTime(startTime) > 0.2) {
            shooter.startShooting();
            shooter.startFeeding(1);
        }
        // commented because handled in end():
        // if (Util.getElaspedTime(startTime) > 1.2) {
        //     shooter.stopFeeding();
        //     shooter.stopShooting();
        // }
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        shooter.stopFeeding();
        shooter.stopShooting();
    }

    @Override
    public boolean isFinished() {
        boolean timeHasElasped = Util.getElaspedTime(startTime) > 1.7;
        return timeHasElasped;// run for 1.7 sec //  || !shooter.hasNote(); // for 1.7 sec OR the note is gone.
    }
}
