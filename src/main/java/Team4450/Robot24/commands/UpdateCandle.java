package Team4450.Robot24.commands;

import static Team4450.Robot24.Constants.alliance;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.Candle;
import Team4450.Robot24.subsystems.Candle.AnimationTypes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Default command that updates the LEDs on the robot!
 */
public class UpdateCandle extends Command {
    private Candle candle;
    private enum State {DISABLED, INTAKING, HAS_NOTE, TARGET_LOCKED, ALLIANCE, OFF}
    private State state = State.DISABLED;
    private double time;

    /**
     * A default command that updates the CANdle LED state, using NetworkTables data
     * @param candle the CANdle subsystem
     */
    public UpdateCandle(Candle candle) {
        this.candle = candle;
        addRequirements(candle);
    }

    @Override // we want this to run while disabled!
    public boolean runsWhenDisabled() {return true;}

    @Override
    public void initialize() {
        time = Util.timeStamp();
    }

    @Override
    public void execute() {
        if (RobotState.isDisabled() && !DriverStation.isFMSAttached()) { // disabled in pit
            state = State.DISABLED;
        } else if (RobotState.isDisabled()) { // disabled on FMS
            state = State.ALLIANCE;
        } else if (SmartDashboard.getBoolean("Intake", false)){ // intaking
            state = State.INTAKING;
        } else if (SmartDashboard.getBoolean("Target Locked", false)) { // target locked
            state = State.TARGET_LOCKED;
        } else if (SmartDashboard.getBoolean("Note Sensor", false)) { // has note
            state = State.HAS_NOTE;
        } else { // no LEDs
            state = State.OFF;
        }
        // Util.consoleLog("state=%s", state.toString());
        setLeds();
    }

    /**
     * Blink the CANdle between two colors
     * @param color the first color
     * @param color2 the second color (or null for off)
     */
    private void blink(Color color, Color color2) {
        if (Util.getElaspedTime(time) % 0.2 < 0.1) {
            candle.setLeds(color);
        } else if (color2 == null) {
            candle.setLedsOff();
        } else {
            candle.setLeds(color2);
        }
    }

    /**
     * Blink the CANdle between the given color and off
     * @param color the color
     */
    private void blink(Color color) {
        blink(color, null);
    }

    /**
     * set the LED colors based on the state
     */
    private void setLeds() {
        switch (state) {
            case DISABLED: // rainbow
                candle.setAnimation(AnimationTypes.Rainbow);
                break;
            case ALLIANCE: // alliance color
                candle.setAnimation(AnimationTypes.Off);
                candle.setLeds(alliance==Alliance.Blue ? Color.kBlue : Color.kRed);
                break;
            case INTAKING: // blinking red
                candle.setAnimation(AnimationTypes.Off);
                blink(Color.kRed);
                break;
            case HAS_NOTE: // solid green
                candle.setAnimation(AnimationTypes.Off);
                candle.setLeds(Color.kGreen);
                break;
            case TARGET_LOCKED: // alternating yellow and blue
                candle.setAnimation(AnimationTypes.Off);
                blink(Color.kBlue, Color.kYellow);
                break;
            case OFF: // LEDs off
                candle.setAnimation(AnimationTypes.Off);
                candle.setLedsOff();
                break;
        }
    }
}
