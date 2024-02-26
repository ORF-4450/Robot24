package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.Elevator;
import Team4450.Robot24.subsystems.Intake;
import Team4450.Robot24.subsystems.Shooter;
import Team4450.Robot24.subsystems.ElevatedShooter.PRESET_POSITIONS;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootAmp extends Command {
    private final ElevatedShooter elevatedShooter;

    private static enum State {MOVING, SHOOTING, DONE};

    private State state = State.MOVING;

    public ShootAmp(ElevatedShooter elevatedShooter) {
        this.elevatedShooter = elevatedShooter;
        addRequirements(elevatedShooter);
    }

    @Override
    public void initialize() {
         state = State.MOVING;
         elevatedShooter.executeSetPosition(PRESET_POSITIONS.SHOOT_AMP_FRONT);
        //  elevatedShooter.shooter.enableClosedLoopFeedStop(true);
    }

    @Override
    public void execute() {
        switch (state) {
            case MOVING:
                if (elevatedShooter.executeSetPosition(PRESET_POSITIONS.SHOOT_AMP_FRONT))
                    state = State.SHOOTING;
                break;
            case SHOOTING:
                elevatedShooter.shooter.startFeeding(-1);
                if (!elevatedShooter.shooter.hasNote())
                    state = State.DONE;
                break;
            case DONE:
                elevatedShooter.shooter.stopFeeding();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }
    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        elevatedShooter.shooter.stopFeeding();
        elevatedShooter.shooter.enableClosedLoopFeedStop(false);
    }
}
