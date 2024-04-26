package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.AdvantageScope;
import Team4450.Robot24.subsystems.ElevatedShooter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Shoots a Note, assuming shooter wheels and angle are already spinning and set.
 */
public class ShootSpeaker extends Command {
    private ElevatedShooter elevatedShooter;
    double startTime;

    /**
     * Shoots a Note, assuming shooter wheels and angle are already spinning and set.
     * @param elevatedShooter the ElevatedShooter subsystem
     */
    public ShootSpeaker(ElevatedShooter elevatedShooter) {
        this.elevatedShooter = elevatedShooter;
    }

    @Override
    public void initialize() {
        // simulate shoot
        if (RobotBase.isSimulation()) AdvantageScope.getInstance().clearGamepieceInventory();
        
        // start feeding the Note into the shooter wheels
        elevatedShooter.shooter.startFeeding(1);
        startTime = Util.timeStamp();
    }

    @Override
    public void execute() {
        // keep feeding the Note, we don't actually run startShooting because
        // they should already be running from spin up command. this allows for multiple
        // shot powers like in AimTrap.
        elevatedShooter.shooter.startFeeding(1);
    }

    @Override
    public boolean isFinished() {
        return (Util.getElaspedTime(startTime) > 0.75); // shoot for 0.75 seconds and then we're done
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        elevatedShooter.shooter.stopFeeding();
        elevatedShooter.shooter.stopShooting();
        elevatedShooter.shooter.hasShot = true; // we set this to cancel other commands like the spinup
        SmartDashboard.putBoolean("Spun Up", false);
    }
}
