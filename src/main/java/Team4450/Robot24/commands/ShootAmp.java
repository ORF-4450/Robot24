package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.Elevator;
import Team4450.Robot24.subsystems.Intake;
import Team4450.Robot24.subsystems.Shooter;
import Team4450.Robot24.subsystems.ElevatedShooter.PresetPosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootAmp extends Command {
    private final ElevatedShooter elevatedShooter;

    private static enum State {MOVING, SHOOTING, MOVING2, DONE};

    private State state = State.MOVING;
    private double feedTime = 0;

    public ShootAmp(ElevatedShooter elevatedShooter) {
        this.elevatedShooter = elevatedShooter;
        addRequirements(elevatedShooter);
        SmartDashboard.putString("ShootAmp Status", state.name());
    }

    @Override
    public void initialize() {
        state = State.MOVING;
         elevatedShooter.executeSetPosition(PresetPosition.SHOOT_AMP_FRONT);
        //  elevatedShooter.shooter.enableClosedLoopFeedStop(true);
    }

    @Override
    public void execute() {
        SmartDashboard.putString("ShootAmp Status", state.name());
        switch (state) {
            case MOVING:
                if (elevatedShooter.executeSetPosition(PresetPosition.SHOOT_AMP_FRONT))
                    state = State.SHOOTING;
                    feedTime = Util.timeStamp();
                break;
            case SHOOTING:
                elevatedShooter.shooter.startFeeding(-0.8);
                if (!elevatedShooter.shooter.hasNote() && Util.getElaspedTime(feedTime) > 0.5)
                    state = State.MOVING2;
                break;
            case MOVING2:
                if (elevatedShooter.executeSetPosition(PresetPosition.SHOOT_AMP_FRONT_TWO))
                    state = State.DONE;
                    feedTime = Util.timeStamp();
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
