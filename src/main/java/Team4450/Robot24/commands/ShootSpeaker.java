package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.AdvantageScope;
import Team4450.Robot24.subsystems.ElevatedShooter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootSpeaker extends Command {
    private ElevatedShooter elevatedShooter;
    double startTime;

    public ShootSpeaker(ElevatedShooter elevatedShooter) {
        this.elevatedShooter = elevatedShooter;
    }

    @Override
    public void initialize() {
        if (RobotBase.isSimulation()) AdvantageScope.getInstance().dropAllNotes();
        // elevatedShooter.shooter.startShooting();
        elevatedShooter.shooter.startFeeding(1);
        startTime = Util.timeStamp();
    }

    @Override
    public void execute() {
        elevatedShooter.shooter.startFeeding(1);
        // elevatedShooter.shooter.startShooting();
    }

    @Override
    public boolean isFinished() {
        return (Util.getElaspedTime(startTime) > 0.75);
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        if (RobotBase.isSimulation()) AdvantageScope.getInstance().dropAllNotes();
        elevatedShooter.shooter.stopFeeding();
        elevatedShooter.shooter.stopShooting();
        elevatedShooter.shooter.hasShot = true;
        SmartDashboard.putBoolean("Spun Up", false);
    }
}
