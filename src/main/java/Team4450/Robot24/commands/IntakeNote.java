package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.AdvantageScope;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.Intake;
import Team4450.Robot24.subsystems.ElevatedShooter.PresetPosition;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Intakes a note by moving to position, and running intake motors until a note appears
 */
public class IntakeNote extends Command {
    private final ElevatedShooter elevatedShooter;
    private final Intake  intake;

    private boolean shooting;

    // this command is one of several that uses multiple States in an enum to keep
    // track of timing / positioning
    private static enum State {MOVING, INTAKING, FEEDING, IN_FINAL_PLACE};
    private State state = State.INTAKING;

    /**
     * Intake a Note by moving to position and running motors until a Note is possessed.
     * 
     * Optionally, it can shoot and intake in one motion!
     * @param intake the Intake subsystem
     * @param elevatedShooter the ElevatedShooter subsystem
     * @param shooting whether to shoot in one action, mostly only for auton to save time
     */
    public IntakeNote(Intake intake, ElevatedShooter elevatedShooter, boolean shooting) {
        this.elevatedShooter = elevatedShooter;
        this.intake = intake;
        this.shooting = shooting;
        
        addRequirements(intake, elevatedShooter);

        // share the state so that we can debug
        SmartDashboard.putString("IntakeNote Status", state.name());
    }

    /**
     * Intake a Note by moving to position and running motors until a Note is possessed.
     * @param intake the Intake subsystem
     * @param elevatedShooter the ElevatedShooter subsystem
     */
    public IntakeNote(Intake intake, ElevatedShooter elevatedShooter) {
        this(intake, elevatedShooter, false);
    }

    @Override
    public void initialize() {
         state = State.MOVING;
         elevatedShooter.executeSetPosition(PresetPosition.INTAKE);
         elevatedShooter.shooter.enableClosedLoopFeedStop(true); // auto stop rollers on intake
         // that uses closed loop control, the sensor is directly connected to Spark Max.
    }

    @Override
    public void execute() {
        SmartDashboard.putString("IntakeNote Status", state.name());
        
        switch (state) {
            case MOVING: // still moving to the intake position, must call executeSetPosition() every loop!
                if (elevatedShooter.executeSetPosition(PresetPosition.INTAKE)) // returns true when done
                    state = State.INTAKING;
                break;
            
            case INTAKING: // waiting for a note and intaking
                intake.start(); // run intake
                elevatedShooter.shooter.startFeeding(1); // run feeder
            
                if (shooting) { // if shooting while intake, then start shooting
                    elevatedShooter.shooter.startShooting();
                } else {
                    // otherwise prevent note from going too far by running shooterwheels
                    // slowly backwards
                    elevatedShooter.shooter.backShoot();
                }
                
                // this attempts to simulate note pickup in simulation, more in AdvantageScope
                boolean simPickup = false;
                
                if (RobotBase.isSimulation()) simPickup = AdvantageScope.getInstance().attemptPickup();

                if (elevatedShooter.shooter.hasNote() || simPickup) { // if the note has been picked up
                    state = State.FEEDING;
                }
                break;

            case FEEDING:// feed until shooter has note, deprecated basically just skip it!
                intake.stop();
                elevatedShooter.shooter.stopFeeding();
                if (!shooting) elevatedShooter.shooter.stopShooting();
                state = State.IN_FINAL_PLACE;
                break;

            case IN_FINAL_PLACE: // pretty much same as FEEDING but we had that if note sensor didn't
                                 // work in case we had to use it on time only
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
        elevatedShooter.shooter.enableClosedLoopFeedStop(false); // always remember to reset this!
    }
}
