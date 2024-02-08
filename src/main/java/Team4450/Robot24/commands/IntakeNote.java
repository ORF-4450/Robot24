package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.Intake;
import Team4450.Robot24.subsystems.Shooter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
    private final Shooter shooter;
    private final Intake  intake;

    private static enum State {INTAKING, FEEDING, IN_SHOOTER};

    private State state = State.INTAKING;
    private double feedTime = 0;

    public IntakeNote(Intake intake, Shooter shooter) {
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        intake.start();
    }

    @Override
    public void execute() {
        switch (state) {
            case INTAKING:
                if (intake.hasNote()) {
                    state = State.FEEDING;
                    intake.start(0.2); // slow it down, not restart it
                    shooter.startFeeding(0.5);
                }
                feedTime = Util.timeStamp();
                break;
            case FEEDING:// feed for 0.5 sec
                if (Util.getElaspedTime(feedTime) > 0.5){//shooter.hasNote()) {
                    state = State.IN_SHOOTER;
                    intake.stop();
                    shooter.stopFeeding();
                }
                break;
            case IN_SHOOTER:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.IN_SHOOTER;
    }
}
