package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.AdvantageScope;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.Intake;
import Team4450.Robot24.subsystems.ElevatedShooter.PresetPosition;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
    private final ElevatedShooter elevatedShooter;
    private final Intake  intake;

    private static enum State {MOVING, INTAKING, FEEDING, IN_FINAL_PLACE};

    private State state = State.INTAKING;
    private double feedTime = 0;

    public IntakeNote(Intake intake, ElevatedShooter elevatedShooter) {
        this.elevatedShooter = elevatedShooter;
        this.intake = intake;
        addRequirements(intake, elevatedShooter);
        SmartDashboard.putString("IntakeNote Status", state.name());

    }

    @Override
    public void initialize() {
         state = State.MOVING;
         elevatedShooter.executeSetPosition(PresetPosition.INTAKE);
         elevatedShooter.shooter.enableClosedLoopFeedStop(true);
    }

    @Override
    public void execute() {
        SmartDashboard.putString("IntakeNote Status", state.name());
        switch (state) {
            case MOVING:
                if (elevatedShooter.executeSetPosition(PresetPosition.INTAKE))
                    state = State.INTAKING;
                break;
            case INTAKING:
                intake.start();
                elevatedShooter.shooter.startFeeding(1);
                elevatedShooter.shooter.backShoot();
                boolean simPickup = false;
                if (RobotBase.isSimulation()) simPickup = AdvantageScope.getInstance().attemptPickup();
                if (elevatedShooter.shooter.hasNote() || simPickup) {
                    state = State.FEEDING;
                    feedTime = Util.timeStamp();
                }
                break;
            case FEEDING:// feed until shooter has note
                intake.stop();
                elevatedShooter.shooter.stopFeeding();
                elevatedShooter.shooter.stopShooting();
                // elevatedShooter.shooter.startFeeding(0.3);
                // if (Util.getElaspedTime(feedTime) > 0.5)
                    state = State.IN_FINAL_PLACE;
                break;
            case IN_FINAL_PLACE:
                intake.stop();

                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.IN_FINAL_PLACE;
    }
    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        intake.stop();
        elevatedShooter.shooter.stopFeeding();
        elevatedShooter.shooter.stopShooting();
        elevatedShooter.shooter.enableClosedLoopFeedStop(false);
    }
}
