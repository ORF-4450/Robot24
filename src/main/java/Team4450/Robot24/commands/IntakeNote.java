package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.Elevator;
import Team4450.Robot24.subsystems.Intake;
import Team4450.Robot24.subsystems.Shooter;
import Team4450.Robot24.subsystems.ElevatedShooter.PRESET_POSITIONS;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
    private final ElevatedShooter elevatedShooter;
    private final Intake  intake;

    private static enum State {MOVING, INTAKING, FEEDING, IN_SHOOTER};

    private State state = State.INTAKING;

    public IntakeNote(Intake intake, ElevatedShooter elevatedShooter) {
        this.elevatedShooter = elevatedShooter;
        this.intake = intake;
        addRequirements(intake, elevatedShooter);
    }

    @Override
    public void initialize() {
         state = State.MOVING;
         elevatedShooter.executeSetPosition(PRESET_POSITIONS.INTAKE);
         elevatedShooter.shooter.enableClosedLoopFeedStop(true);
    }

    @Override
    public void execute() {
        switch (state) {
            case MOVING:
                if (elevatedShooter.executeSetPosition(PRESET_POSITIONS.INTAKE))
                    state = State.INTAKING;
                break;
            case INTAKING:
                intake.start();
                elevatedShooter.shooter.startFeeding(1);
                // // once the intake has the note, slow it down
                // if (intake.hasNote()) {
                //     shooter.note = true;
                //     state = State.FEEDING;
                //     intake.start(0.9); // slow it down a little
                // }
                // feedTime = Util.timeStamp();
                break;
            case FEEDING:// feed until shooter has note
                if (elevatedShooter.shooter.hasNote()){//Util.getElaspedTime(feedTime) > 3){//shooter.hasNote()) {
                    intake.stop();
                    elevatedShooter.shooter.stopFeeding();
                    state = State.IN_SHOOTER;
                }
                break;
            case IN_SHOOTER:
                intake.stop();
                elevatedShooter.shooter.stopFeeding();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.IN_SHOOTER;
    }
    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        intake.stop();
        elevatedShooter.shooter.stopFeeding();
        elevatedShooter.shooter.enableClosedLoopFeedStop(false);
    }
}
