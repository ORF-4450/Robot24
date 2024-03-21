package Team4450.Robot24.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatedShooter extends SubsystemBase {
    public final Shooter       shooter;
	public final Elevator      elevator;


    public static enum PresetPosition {INTAKE, SHOOT, SHOOT_VISION_START, SHOOT_AMP_FRONT, SHOOT_PODIUM_HIGH, SHOOT_AMP_BACK,SHOOT_AMP_FRONT_TWO, VERTICAL_BOTTOM, VERTICAL_TOP, SOURCE, CLIMB, NONE};

    public boolean shootDoesTheSpeakerInsteadOfTheAmp = true;

    private double elevatorHeight;
    private double pivotAngle;
    private double centerstageHeight;
    private boolean atTop;
    private PresetPosition position = PresetPosition.NONE;

    // in meters and degrees
    private final double CENTERSTAGE_SAFE_TOP = 0.4;
    private final double CENTERSTAGE_SAFE_BOTTOM = 0;
    private final double MAIN_SAFE_TOP = 0.62;
    private final double MAIN_SAFE_BOTTOM = 0.2;
    private final double PIVOT_SAFE = -90; // angle okay to move up/down
    

    /**
     * MUST be called in the command scheduler loop via execute() or periodic()
     * @param position the preset position for the shooter and elevator combo
     * @return whether or not it is in position (true), or there is still more work to do (false)
     */
    public boolean executeSetPosition(PresetPosition position) {
        this.position = position;
        switch (position) {
            // set target position/rotation values for each position
            case INTAKE:
                pivotAngle = -39;
                elevatorHeight = 0.114;
                centerstageHeight = CENTERSTAGE_SAFE_BOTTOM;
                atTop = false;
                break;
            case CLIMB:
                pivotAngle = -160;
                elevatorHeight = MAIN_SAFE_TOP;
                centerstageHeight = CENTERSTAGE_SAFE_TOP;
                atTop = true;
                break;
            case SHOOT:
                pivotAngle = -39;
                elevatorHeight = 0.15;
                centerstageHeight = CENTERSTAGE_SAFE_BOTTOM;
                atTop = false;
                break;
            case SHOOT_VISION_START:
                pivotAngle = -27;
                elevatorHeight = 0.2;
                centerstageHeight = CENTERSTAGE_SAFE_BOTTOM;
                atTop = false;
                break;
            case SHOOT_AMP_FRONT:
                pivotAngle = -3;
                elevatorHeight = MAIN_SAFE_TOP;
                centerstageHeight = CENTERSTAGE_SAFE_TOP;
                atTop = true;
                break;
            case SHOOT_PODIUM_HIGH:
                pivotAngle = -20;
                elevatorHeight = MAIN_SAFE_TOP;
                centerstageHeight = CENTERSTAGE_SAFE_TOP;
                atTop = true;
                break;
            case SHOOT_AMP_FRONT_TWO:
                pivotAngle = 15;
                elevatorHeight = MAIN_SAFE_TOP;
                centerstageHeight = CENTERSTAGE_SAFE_TOP;
                atTop = true;
                break;
            
            case SHOOT_AMP_BACK: // no=========================
                pivotAngle = -150;
                elevatorHeight = MAIN_SAFE_TOP;
                centerstageHeight = CENTERSTAGE_SAFE_TOP;
                atTop = true;
                break;
            case VERTICAL_BOTTOM:
                pivotAngle = PIVOT_SAFE;
                elevatorHeight = MAIN_SAFE_BOTTOM;
                centerstageHeight = CENTERSTAGE_SAFE_BOTTOM;
                atTop = false;
                break;
            case VERTICAL_TOP:
                pivotAngle = PIVOT_SAFE;
                elevatorHeight = MAIN_SAFE_TOP;
                centerstageHeight = CENTERSTAGE_SAFE_TOP;
                atTop = true;
                break;
            case SOURCE:
                pivotAngle = -180;
                elevatorHeight = MAIN_SAFE_TOP;
                centerstageHeight = CENTERSTAGE_SAFE_TOP;
                atTop = true;
                break;
            case NONE:
                break;
        }

        return execute();
    }

    public boolean executeSetPosition(double pivotAngle, double centerstageHeight, double elevatorHeight, boolean topHalf) {
        this.position = PresetPosition.NONE;
        this.pivotAngle = pivotAngle;
        this.centerstageHeight = centerstageHeight;
        this.elevatorHeight = elevatorHeight;
        this.atTop = topHalf;
        return execute();
    }

    public boolean execute() {
        // actual movement
        double safeTargetMainHeight = atTop ? MAIN_SAFE_TOP : MAIN_SAFE_BOTTOM;
        double safeTargetCenterstageHeight = atTop ? CENTERSTAGE_SAFE_TOP : CENTERSTAGE_SAFE_BOTTOM;
        double safeOtherMainHeight = !atTop ? MAIN_SAFE_TOP : MAIN_SAFE_BOTTOM;
        double safeOtherCenterstageHeight = !atTop ? CENTERSTAGE_SAFE_TOP : CENTERSTAGE_SAFE_BOTTOM;

        if (isAtTopHalf() == atTop) {
            if (shooter.isAtAngle(pivotAngle)) {
                if (centerstageIsAtHeight(centerstageHeight) && elevatorIsAtHeight(elevatorHeight)) {
                    elevator.setCenterstageHeight(centerstageHeight);
                    elevator.setElevatorHeight(elevatorHeight);
                    elevator.move(0);
                    elevator.moveCenterStage(0);
                    SmartDashboard.putString("position_step", "A");
                    return true;
                } else {
                    SmartDashboard.putString("position_step", "B");
                    elevator.setCenterstageHeight(centerstageHeight);
                    elevator.setElevatorHeight(elevatorHeight);
                }
            } else if (centerstageIsAtHeight(safeTargetCenterstageHeight) && elevatorIsAtHeight(safeTargetMainHeight)) {
                SmartDashboard.putString("position_step", "C");
                elevator.setCenterstageHeight(safeTargetCenterstageHeight);
                elevator.setElevatorHeight(safeTargetMainHeight);
                shooter.setAngle(pivotAngle);
            } else {
                SmartDashboard.putString("position_step", "D");
                elevator.setCenterstageHeight(safeTargetCenterstageHeight);
                elevator.setElevatorHeight(safeTargetMainHeight);
            }
        } else {
            if (shooter.isAtAngle(PIVOT_SAFE)) {
                SmartDashboard.putString("position_step", "E");
                elevator.setCenterstageHeight(safeTargetCenterstageHeight);
                elevator.setElevatorHeight(safeTargetMainHeight);
                shooter.setAngle(PIVOT_SAFE);
            } else if (centerstageIsAtHeight(safeOtherCenterstageHeight) && elevatorIsAtHeight(safeOtherMainHeight)) {
                SmartDashboard.putString("position_step", "F");
                elevator.setCenterstageHeight(safeOtherCenterstageHeight);
                elevator.setElevatorHeight(safeOtherMainHeight);
                shooter.setAngle(PIVOT_SAFE);
            } else {
                SmartDashboard.putString("position_step", "G");
                elevator.setCenterstageHeight(safeOtherCenterstageHeight);
                elevator.setElevatorHeight(safeOtherMainHeight);
            }
        }
        return false;
    }
       
    public ElevatedShooter() {
        shooter = new Shooter();
		elevator = new Elevator();
        SmartDashboard.putString("position_step", "_");
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Preset Position", position.name());
    }

    // private boolean safeToPivot() {
    //     boolean centerstageSafeTop = centerstageIsAtHeight(CENTERSTAGE_SAFE_TOP);
    //     boolean centerstageSafeBottom = centerstageIsAtHeight(CENTERSTAGE_SAFE_BOTTOM);
    //     boolean mainSafeTop = elevatorIsAtHeight(MAIN_SAFE_TOP);
    //     boolean mainSafeBottom = elevatorIsAtHeight(MAIN_SAFE_BOTTOM);
    //     return (centerstageSafeTop && mainSafeTop) || (centerstageSafeBottom && mainSafeBottom);    
    // }

    public void setUnsafeRelativePosition(double pivotAngleChange, double centerstageHeightChange, double elevatorHeightChange) {
        shooter.movePivotRelative(pivotAngleChange);
        elevator.move(elevatorHeightChange * 0.9);
        elevator.moveCenterStage(centerstageHeightChange);
    }

    private boolean isAtTopHalf() {
        return elevator.getElevatorHeight() > 0.4;
    }

    private boolean centerstageIsAtHeight(double height) {
        return true;
        // return elevator.centerstageIsAtHeight(height);
    }
    private boolean elevatorIsAtHeight(double height) {
        return elevator.elevatorIsAtHeight(height);
    }
    


    // @Override
    // public void periodic() {
  
    // }

    // public boolean isAtPosition() {
    //     return false;
    // }

    public boolean hasNote() {return shooter.hasNote();}
    public void resetEncoders() {shooter.resetEncoders();elevator.resetEncoders();}
}
