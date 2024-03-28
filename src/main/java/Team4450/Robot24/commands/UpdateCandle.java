package Team4450.Robot24.commands;

import static Team4450.Robot24.Constants.alliance;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.Candle;
import Team4450.Robot24.subsystems.Candle.AnimationTypes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class UpdateCandle extends Command {
    private Candle candle;
    private enum State {DISABLED, INTAKING, HAS_NOTE, TARGET_LOCKED, ALLIANCE, OFF}
    private State state = State.DISABLED;
    private double time;

    public UpdateCandle(Candle candle) {
        this.candle = candle;
        addRequirements(candle);
    }

    @Override
    public boolean runsWhenDisabled() {return true;}

    @Override
    public void initialize() {
        time = Util.timeStamp();
        // candle.setSpeed(0.1);
    }

    @Override
    public void execute() {
        if (RobotState.isDisabled()) {
            state = State.DISABLED;
        } else if (RobotState.isAutonomous()) {
            state = State.ALLIANCE;
        } else if (SmartDashboard.getBoolean("Intake", false)){
            state = State.INTAKING;
        } else if (SmartDashboard.getBoolean("Target Locked", false)) {
            state = State.TARGET_LOCKED;
        } else if (SmartDashboard.getBoolean("Note Sensor", false)) {
            state = State.HAS_NOTE;
        } else {
            state = State.OFF;
        }
        // Util.consoleLog("state=%s", state.toString());
        setLeds();
    }

    private void blink(Color color) {
        if (Util.getElaspedTime(time) % 0.5 < 0.3) {
            candle.setLeds(color);
        } else {
            candle.setLedsOff();
        }
    }

    private void setLeds() {
        switch (state) {
            case DISABLED:
                candle.setAnimation(AnimationTypes.Rainbow);
                break;
            case ALLIANCE:
                candle.setAnimation(AnimationTypes.Off);
                candle.setLeds(alliance==Alliance.Blue ? Color.kBlue : Color.kRed);
                break;
            case INTAKING:
                candle.setAnimation(AnimationTypes.Off);
                blink(Color.kRed);
                break;
            case HAS_NOTE:
                candle.setAnimation(AnimationTypes.Off);
                candle.setLeds(Color.kGreen);
                break;
            case TARGET_LOCKED:
                candle.setAnimation(AnimationTypes.Off);
                blink(Color.kGreen);
                break;
            case OFF:
                candle.setAnimation(AnimationTypes.Off);
                candle.setLedsOff();
                break;
        }
    }
}
