package Team4450.Robot24.subsystems;

import static Team4450.Robot24.Constants.TRAP_ANGLE;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A combination subsyetsm that controls the Shooter and Elevator. This is used
 * because of the close relation between where it's safe to pivot and extend.
 * This class basically does all that checking and moving for you so you can just give
 * it values and it moves SAFELY (theoretically)
 */
public class ElevatedShooter extends SubsystemBase {
    // these are both publicly accessible to allow passthrough for things like
    // shoot commands, but we are trusting commads not to abuse this by moving
    // the actual mechanisms (pivot/elevator)
    public final Shooter       shooter;
	public final Elevator      elevator;

    /** an enum that represents several Preset positions of the Elevator and Shooter */
    public static enum PresetPosition {
        /** the position to start intaking at */ INTAKE,
        /** the position to start shooting at */ SHOOT,
        /** the position to start Trap at */ TRAP,
        /** the position to start tracking at */ SHOOT_VISION_START,
        /** the position to start shooting Amp at */ SHOOT_AMP_FRONT, 
        /** the position to start shooting at a high shot position */ HIGH_SHOT, 
        /** the position to start reverse Amp at */ SHOOT_AMP_BACK,
        /** the position to start the second Amp scoring sequence */ SHOOT_AMP_FRONT_TWO, 
        /** the position to move to vertical shooter at zero extension */ VERTICAL_BOTTOM, 
        /** the position to move to vertical shooter at high extension */ VERTICAL_TOP, 
        /** the position to start intaking from the Source at */ SOURCE, 
        /** the position to start climbing at */ CLIMB, 
        /** No position */ NONE,
        WING_SHOT
    };

    // NOTE: all elevator heights in THIS java file (different in Elevator.java)
    // are in "meters" (not quite exactly meters but close enough...). Same deal
    // with the pivot angles (but in degrees of course)

    /** whether the "score" button (RB) does the Speaker or Amp shoot sequence */
    public boolean shootDoesTheSpeakerInsteadOfTheAmp = true;

    /** the end goal of the elevator height: not the setpoint but the eventual preset goal */
    private double endGoalElevatorHeight;

    /** the end goal of the pivot angle: not the setpoint but the eventual preset goal */
    private double endGoalPivotAngle;

    /** whether the end goal is at the top half or bottom half (crosses the crossbar) */
    private boolean atTop;
    private PresetPosition position = PresetPosition.NONE;

    // in meters and degrees
    /** the safe extension height to pivot at top half */
    private final double MAIN_SAFE_TOP = 0.62;

    /** the safe extension height to pivot at lower half */
    private final double MAIN_SAFE_BOTTOM = 0.16;

    /** the safe pivot angle to extend at */
    private final double PIVOT_SAFE = -90; // angle okay to move up/down
    

    /**
     * Sets the end goal state via a PresetPosition. Unless you are calling execute()
     * seperately, this MUST be called in the command scheduler loop regularly
     * @param position the preset position for the shooter and elevator combo
     * @return whether or not it is in position (true), or there is still more work to do (false)
     */
    public boolean executeSetPosition(PresetPosition position) {
        this.position = position;
        switch (position) {
            
            // set target position/rotation values for each position
            case INTAKE:
                endGoalPivotAngle = -39;
                endGoalElevatorHeight = 0.09;
                atTop = false;
                break;
            case CLIMB:
                endGoalPivotAngle = -160;
                endGoalElevatorHeight = MAIN_SAFE_TOP;
                atTop = true;
                break;
            case WING_SHOT:
                endGoalPivotAngle = -33.3;
                endGoalElevatorHeight = 0.108;
                atTop = false;
                break;
            case SHOOT:
                endGoalPivotAngle = -39;
                endGoalElevatorHeight = 0.15;
                atTop = false;
                break;
            case SHOOT_VISION_START:
                endGoalPivotAngle = -39;
                endGoalElevatorHeight = 0.15;
                atTop = false;
                break;
            case TRAP:
                endGoalPivotAngle = TRAP_ANGLE;
                endGoalElevatorHeight = 0.15;
                atTop = false;
                break;
            case SHOOT_AMP_FRONT:
                endGoalPivotAngle = -6;
                endGoalElevatorHeight = MAIN_SAFE_TOP;
                atTop = true;
                break;
            case HIGH_SHOT:
                endGoalPivotAngle = -20;
                endGoalElevatorHeight = MAIN_SAFE_TOP;
                atTop = true;
                break;
            case SHOOT_AMP_FRONT_TWO:
                endGoalPivotAngle = 15;
                endGoalElevatorHeight = MAIN_SAFE_TOP;
                atTop = true;
                break;
            case SHOOT_AMP_BACK: // not using
                endGoalPivotAngle = -150;
                endGoalElevatorHeight = MAIN_SAFE_TOP;
                atTop = true;
                break;
            case VERTICAL_BOTTOM:
                endGoalPivotAngle = PIVOT_SAFE;
                endGoalElevatorHeight = MAIN_SAFE_BOTTOM;
                atTop = false;
                break;
            case VERTICAL_TOP:
                endGoalPivotAngle = PIVOT_SAFE;
                endGoalElevatorHeight = MAIN_SAFE_TOP;
                atTop = true;
                break;
            case SOURCE:
                endGoalPivotAngle = -120;
                endGoalElevatorHeight = MAIN_SAFE_TOP;
                atTop = true;
                break;
            case NONE:
                break;
        }

        return execute();
    }

    /**
     * Sets the end goal state via a PresetPosition. Unless you are calling execute()
     * seperately, this MUST be called in the command scheduler loop regularly
     * @param pivotAngle the desired end pivot angle (degrees)
     * @param elevatorHeight the desired end elevator height (meters)
     * @param topHalf whether the endo goal is at top half (true) or not (false)
     * @return whether or not it is in position (true), or there is still more work to do (false)
     */
    public boolean executeSetPosition(double pivotAngle, double elevatorHeight, boolean topHalf) {
        this.position = PresetPosition.NONE;
        this.endGoalPivotAngle = pivotAngle;
        this.endGoalElevatorHeight = elevatorHeight;
        this.atTop = topHalf;
        return execute();
    }

    /**
     * Check if the system is at the end state, and if it's not: set the next goal/setpoint
     * in order to achive the goal. This automatically breaks up the setpoints so that they
     * are sequential and "safe" so that nothing hits anything else.
     * @apiNote NOTE that it's probably best not to use this directly, and call
     * either of executeSetPosition()'s methods because they call this interally
     * @return
     */
    public boolean execute() {
        /** the safe height to pivot at end goal (choose between MAIN_SAFE_TOP or _BOTTOM for end goal) */
        double safeTargetMainHeight = atTop ? MAIN_SAFE_TOP : MAIN_SAFE_BOTTOM;
        /** the safe height to pivot at intermediate goal (choose between MAIN_SAFE_TOP or _BOTTOM for intermediate goal) */
        double safeOtherMainHeight = !atTop ? MAIN_SAFE_TOP : MAIN_SAFE_BOTTOM;

        // what follows is a whole bunch of logic that makes safe travel possible.
        // it's kind confusing to follow, but I promise you it DOES WORK. I've provided
        // a flowchart+diagram at the link below for people to look at and understand this -cole
        // https://docs.google.com/drawings/d/1LPBGhWrGQfPFZmyQl5N3DZTWH7uCNGu50NGBOTV1PWI/edit

        if (isAtTopHalf() == atTop) { // check if we are already at the correct half
            if (shooter.isAtAngle(endGoalPivotAngle)) {
                if (elevatorIsAtHeight(endGoalElevatorHeight)) {
                    // the pivot is at the correct end goal, and so is the elevator
                    // we're done!
                    elevator.setElevatorHeight(endGoalElevatorHeight);
                    elevator.move(0); // TODO: I don't understand/remember why I put this here -cole 3/23/24
                    // this NT data is for debugging, letters kind of arbitrary but in order of line:
                    SmartDashboard.putString("position_step", "A");
                    return true;
                } else {
                    // the elevator is at the safe pivot height and the
                    // pivot has pivoted to end state, but we still need
                    // to move elevator to target end height
                    SmartDashboard.putString("position_step", "B");
                    elevator.setElevatorHeight(endGoalElevatorHeight);
                }
            } else if (elevatorIsAtHeight(safeTargetMainHeight)) {
                // we are at the correct height to pivot to final angle at
                // (not necessarily final height), so we pivot here
                SmartDashboard.putString("position_step", "C");
                elevator.setElevatorHeight(safeTargetMainHeight);
                shooter.setAngle(endGoalPivotAngle);
            } else {
                // the elevator is at the correct half, but it's not yet
                // at a place where it can safely pivot, so we go to the
                // safe pivot height for the correct half
                SmartDashboard.putString("position_step", "D");
                elevator.setElevatorHeight(safeTargetMainHeight);
            }
        } else { // at the wrong half of the extension for our goal
            if (shooter.isAtAngle(PIVOT_SAFE)) {
                // we are at the safe pivot angle to climb past the crossbar,
                // so we do that now
                SmartDashboard.putString("position_step", "E");
                elevator.setElevatorHeight(safeTargetMainHeight);
                shooter.setAngle(PIVOT_SAFE);
            } else if (elevatorIsAtHeight(safeOtherMainHeight)) {
                // we are at the wrong half, and the pivot isn't quite ready to
                // climb past the crossbar (either up or down), so we make it
                // safe to do that now because we are at the safe pivot height
                SmartDashboard.putString("position_step", "F");
                elevator.setElevatorHeight(safeOtherMainHeight);
                shooter.setAngle(PIVOT_SAFE);
            } else {
                // we are at the wrong half, and not at a safe place to pivot to
                // the vertical clib position yet, so we move to that position
                // now
                SmartDashboard.putString("position_step", "G");
                elevator.setElevatorHeight(safeOtherMainHeight);
            }
        }
        return false;
    }
    
    /**
     * A combination subsyetsm that controls the Shooter and Elevator. This is used
     * because of the close relation between where it's safe to pivot and extend.
     * This class basically does all that checking and moving for you so you can just give
     * it values and it moves SAFELY (theoretically)
     */
    public ElevatedShooter() {
        shooter = new Shooter();
		elevator = new Elevator();
        SmartDashboard.putString("position_step", "_");
    }

    @Override
    public void periodic() {
        // we log the name of the preset position to look at for debugging
        SmartDashboard.putString("Preset Position", position.name());
    }

    /**
     * Move the pivot and shooter setpoints/goals relative to the current setpoint/goal
     * @param pivotAngleChange joystick (degree) increment/decrement
     * @param elevatorHeightChange joystick increment/decrement
     */
    public void moveRelative(double pivotAngleChange, double elevatorHeightChange) {
        shooter.movePivotRelative(pivotAngleChange);
        elevator.move(elevatorHeightChange * 0.9);
    }

    /**
     * whether the carriage/shooter is currently at the top half of the elevator or not
     * @return true or false
     */
    private boolean isAtTopHalf() {
        return elevator.getElevatorHeight() > 0.4;
    }

    /**
     * Passthrough function to elevator.elevatorIsAtHeight 
     * @param height the height to check
     * @return if it's at the height (within tolerance)
     */
    private boolean elevatorIsAtHeight(double height) {
        return elevator.elevatorIsAtHeight(height);
    }

    /**
     * passthrough function to shooter.hasNote
     * @return whether a note is possessed
     */
    public boolean hasNote() {return shooter.hasNote();}

    /**
     * reset both the shooter and elevator encoders
     */
    public void resetEncoders() {shooter.resetEncoders();elevator.resetEncoders();}
}
