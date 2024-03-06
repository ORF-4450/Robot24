package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.ElevatedShooter;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootSpeaker extends Command {
    private ElevatedShooter elevatedShooter;
    double startTime;

    public ShootSpeaker(ElevatedShooter elevatedShooter) {
        this.elevatedShooter = elevatedShooter;
    }

    @Override
    public void initialize() {
        elevatedShooter.shooter.startShooting();
        elevatedShooter.shooter.startFeeding(-0.3);
        startTime = Util.timeStamp();
    }

    @Override
    public void execute() {
        if (Util.getElaspedTime(startTime) > 0.3) {
            elevatedShooter.shooter.startFeeding(1);
        }
        elevatedShooter.shooter.startShooting();
    }

    @Override
    public boolean isFinished() {
        return (Util.getElaspedTime(startTime) > 2);
    }

    @Override
    public void end(boolean interrupted) {
        elevatedShooter.shooter.stopFeeding();
        elevatedShooter.shooter.stopShooting();
    }
}
