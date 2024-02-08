package Team4450.Robot24.commands;

import Team4450.Robot24.subsystems.Intake;
import Team4450.Robot24.subsystems.Shooter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
    private final Shooter shooter;
    private final Intake  intake;
    private final DigitalInput intakeNoteSensor = new DigitalInput(0);
    private final DigitalInput shooterNoteSensor = new DigitalInput(1);


    private static enum State {INTAKING, FEEDING, IN_SHOOTER};

    private State state = State.INTAKING;

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
                if (intakeNoteSensor.get()) {
                    state = State.FEEDING;
                    intake.start(0.2); // slow it down
                    shooter.startFeeding(0.5);
                }
                break;
            case FEEDING:
                if (shooterNoteSensor.get()) {
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
