package Team4450.Robot24.subsystems;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatedShooter extends SubsystemBase {
    public final Shooter       shooter;
	public final Elevator      elevator;

    public static enum PRESET_POSITIONS {INTAKE, SHOOT_AMP_FRONT, SHOOT_AMP_BACK, VERTICAL_BOTTOM, VERTICAL_TOP, SOURCE};

    private double elevatorHeight;
    private double pivotAngle;
    private double centerstageHeight;
    private boolean atTop;

    // in meters and degrees
    private final double CENTERSTAGE_SAFE_TOP = 0.4;
    private final double CENTERSTAGE_SAFE_BOTTOM = 0;
    private final double MAIN_SAFE_TOP = 0.7;
    private final double MAIN_SAFE_BOTTOM = 0.3;
    private final double PIVOT_SAFE = 90; // angle okay to move up/down
    

    /**
     * MUST be called in the command scheduler loop via execute() or periodic()
     * @param position the preset position for the shooter and elevator combo
     * @return whether or not it is in position (true), or there is still more work to do (false)
     */
    public boolean executeSetPosition(PRESET_POSITIONS position) {
        switch (position) {
            // set target position/rotation values for each position
            case INTAKE:
                pivotAngle = 45;
                elevatorHeight = 0.15;
                centerstageHeight = CENTERSTAGE_SAFE_BOTTOM;
                atTop = false;
                break;
            case SHOOT_AMP_FRONT:
                pivotAngle = 0;
                elevatorHeight = MAIN_SAFE_TOP  + 0.2;
                centerstageHeight = CENTERSTAGE_SAFE_TOP;
                atTop = true;
                break;
            case SHOOT_AMP_BACK:
                pivotAngle = -180;
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
        }

        return execute();
    }

    public boolean executeSetPosition(double pivotAngle, double centerstageHeight, double elevatorHeight, boolean topHalf) {
        this.pivotAngle = pivotAngle;
        this.centerstageHeight = centerstageHeight;
        this.elevatorHeight = elevatorHeight;
        this.atTop = topHalf;
        return execute();
    }

    private boolean execute() {
        // actual movement
        double safeTargetMainHeight = atTop ? MAIN_SAFE_TOP : MAIN_SAFE_BOTTOM;
        double safeTargetCenterstageHeight = atTop ? CENTERSTAGE_SAFE_TOP : CENTERSTAGE_SAFE_BOTTOM;
        double safeOtherMainHeight = !atTop ? MAIN_SAFE_TOP : MAIN_SAFE_BOTTOM;
        double safeOtherCenterstageHeight = !atTop ? CENTERSTAGE_SAFE_TOP : CENTERSTAGE_SAFE_BOTTOM;

        Util.consoleLog("%f", safeTargetMainHeight);
        if (isAtTopHalf() == atTop) {
            if (shooter.isAtAngle(pivotAngle)) {
                if (centerstageIsAtHeight(centerstageHeight) && elevatorIsAtHeight(elevatorHeight)) {
                    Util.consoleLog("A");
                    return true;
                } else {
                    Util.consoleLog("B");
                    elevator.setCenterstageHeight(centerstageHeight);
                    elevator.setElevatorHeight(elevatorHeight);
                }
            } else if (centerstageIsAtHeight(safeTargetCenterstageHeight) && elevatorIsAtHeight(safeTargetMainHeight)) {
                Util.consoleLog("C");
                shooter.setAngle(pivotAngle);
            } else {
                Util.consoleLog("D");
                elevator.setCenterstageHeight(safeTargetCenterstageHeight);
                elevator.setElevatorHeight(safeTargetMainHeight);
            }
        } else {
            if (shooter.isAtAngle(PIVOT_SAFE)) {
                Util.consoleLog("E");
                elevator.setCenterstageHeight(safeTargetCenterstageHeight);
                elevator.setElevatorHeight(safeTargetMainHeight);
            } else if (centerstageIsAtHeight(safeOtherCenterstageHeight) && elevatorIsAtHeight(safeOtherMainHeight)) {
                Util.consoleLog("F");
                shooter.setAngle(PIVOT_SAFE);
            } else {
                Util.consoleLog("G");
                elevator.setCenterstageHeight(safeOtherCenterstageHeight);
                elevator.setElevatorHeight(safeOtherMainHeight);
            }
        }
        return false;
    }
       
    public ElevatedShooter() {
        shooter = new Shooter();
		elevator = new Elevator();
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
        elevator.move(elevatorHeightChange);
        elevator.moveCenterStage(centerstageHeightChange);
    }

    private boolean isAtTopHalf() {
        return elevator.getElevatorHeight() > 0.3;
    }

    private boolean centerstageIsAtHeight(double height) {
        return Math.abs(elevator.getCenterstageHeight() - height) < 0.02;
    }
    private boolean elevatorIsAtHeight(double height) {
        return Math.abs(elevator.getElevatorHeight() - height) < 0.02;
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
