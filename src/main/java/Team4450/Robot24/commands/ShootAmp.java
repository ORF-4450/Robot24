package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.ElevatedShooter.PresetPosition;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Executes the shoot sequence for the Amp, assuming we are already in Amp
 * position. Important that we do that first!
 */
public class ShootAmp extends Command {
    private final ElevatedShooter elevatedShooter;

    private static enum State {NONE, SHOOTING, MOVING2,MOVING3, DONE};

    private State state = State.NONE;
    private double feedTime = 0;

    /**
     * Executes the shoot sequence for the Amp, assuming we are already in Amp
     * position. Important that we do that first!
     * @param elevatedShooter the ElevatedShooter subsystem
     */
    public ShootAmp(ElevatedShooter elevatedShooter) {
        this.elevatedShooter = elevatedShooter;
        addRequirements(elevatedShooter);
        SmartDashboard.putString("ShootAmp Status", state.name());
    }

    @Override
    public void initialize() {
        state = State.SHOOTING;
        feedTime = Util.timeStamp();
    }

    @Override
    public void execute() {
        SmartDashboard.putString("ShootAmp Status", state.name());
        
        switch (state) {
            case NONE:
                break;
        
            case SHOOTING: // start the initial shoot maneuver
                elevatedShooter.shooter.startFeeding(-0.3);
                boolean moveon = !elevatedShooter.shooter.hasNote() && Util.getElaspedTime(feedTime) > 0.2;
                if (RobotBase.isSimulation()) moveon = true;
                if (moveon) state = State.MOVING2;
                break;

            case MOVING2: // move the pivot to the second position
                if (elevatedShooter.executeSetPosition(PresetPosition.SHOOT_AMP_FRONT_TWO))
                    state = State.MOVING3;
                
                feedTime = Util.timeStamp();
                break;

            case MOVING3: // go back to intake
                if (elevatedShooter.executeSetPosition(PresetPosition.INTAKE))
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
        
        // we reset this boolean so that the "score" button goes back to doing speaker
        // instead of Amp. Very important!
        elevatedShooter.shootDoesTheSpeakerInsteadOfTheAmp = true;
    }
}
