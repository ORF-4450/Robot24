package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.Elevator;
import Team4450.Robot24.subsystems.Intake;
import Team4450.Robot24.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
    private final Shooter shooter;
    private final Elevator elevator;
    private final Intake  intake;

    private static enum State {INTAKING, FEEDING, IN_SHOOTER};

    private State state = State.INTAKING;
    private double feedTime = 0;
    private double intakeTime = 0;

    public IntakeNote(Intake intake, Shooter shooter, Elevator elevator) {
        this.shooter = shooter;
        this.intake = intake;
        this.elevator = elevator;
        addRequirements(shooter, intake, elevator);
    }

    @Override
    public void initialize() {
        intake.start();
        shooter.startFeeding(1);
        intakeTime = Util.timeStamp();
        state = State.INTAKING;
    }

    @Override
    public void execute() {
        switch (state) {
            case INTAKING:
                // once the intake has the note, slow it down
                if (intake.hasNote()) {
                    shooter.note = true;
                    state = State.FEEDING;
                    intake.start(0.9); // slow it down a little
                }
                feedTime = Util.timeStamp();
                break;
            case FEEDING:// feed for 3 sec
                if (intake.hasNote()){//Util.getElaspedTime(feedTime) > 3){//shooter.hasNote()) {
                    intake.stop();
                    shooter.stopFeeding();
                    state = State.IN_SHOOTER;
                }
                break;
            case IN_SHOOTER:
                intake.stop();
                shooter.stopFeeding();
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
        shooter.stopFeeding();
    }
}
