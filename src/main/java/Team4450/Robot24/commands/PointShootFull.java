package Team4450.Robot24.commands;

import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import Team4450.Robot24.AdvantageScope;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.Elevator;
import Team4450.Robot24.subsystems.Shooter;
import Team4450.Robot24.subsystems.ElevatedShooter.PresetPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class PointShootFull extends Command {
    private final DriveBase robotDrive;
    private final ElevatedShooter elevatedShooter;
    private double startTime;
    private  enum State {NONE, MOVING, BACKFEED, SHOOT, DONE};
    private State state = State.NONE;

    private double angle;
    private boolean justShoot = false;
    private boolean manualAngle;

    public PointShootFull(ElevatedShooter elevatedShooter, DriveBase robotDrive, double angle) {
        SmartDashboard.putString("ShootSpeaker Status", state.name());
        this.elevatedShooter = elevatedShooter;
        this.robotDrive = robotDrive;
        this.manualAngle = false;
        this.angle = angle;
        addRequirements(elevatedShooter);
    }
    public PointShootFull(ElevatedShooter elevatedShooter, DriveBase robotDrive) {
        this(elevatedShooter, robotDrive, Double.NaN);
    }

    public PointShootFull(ElevatedShooter elevatedShooter, DriveBase robotDrive, boolean justShoot) {
        SmartDashboard.putString("ShootSpeaker Status", state.name());
        this.elevatedShooter = elevatedShooter;
        this.robotDrive = robotDrive;
        this.manualAngle = true;
        this.justShoot = justShoot;
        this.angle = 0;
        addRequirements(elevatedShooter);
    }

    // pivotAngle = -39;
    // elevatorHeight = 0.15;
    // centerstageHeight = CENTERSTAGE_SAFE_BOTTOM;
    // atTop = false;
    // break;
    @Override
    public void initialize() {
        elevatedShooter.shooter.enableClosedLoopFeedStop(false);
        
        if (justShoot) {
            state = State.BACKFEED;
            startTime = Util.timeStamp();
        }
        else
            state = State.MOVING;
    }

    @Override
    public void execute() {
        SmartDashboard.putString("ShootSpeaker Status", state.name());
        switch (state) {
            case NONE:
                break;
            case MOVING:
                double angleSetpoint;
                if (Double.isNaN(angle)) {
                    angleSetpoint = calculateAngle();
                } else {
                    angleSetpoint = angle;
                }
                if (elevatedShooter.executeSetPosition(angleSetpoint, 0, 0.15, false)) {
                    state = State.BACKFEED;
                    startTime = Util.timeStamp();
                }
                break;
            case BACKFEED:
                if (Util.getElaspedTime(startTime) < 0.1) {
                    elevatedShooter.shooter.startFeeding(-0.3); // start by feeding the note backwards a bit (30% speed for 0.2 seconds see down below)
                    elevatedShooter.shooter.startShooting();
                    // elevatedShooter.shooter.backShoot();
                }
                else if (Util.getElaspedTime(startTime) > 1.0) {
                    state = State.SHOOT;
                    startTime = Util.timeStamp();
                } else {
                    elevatedShooter.shooter.startFeeding(0); // start by feeding the note backwards a bit (30% speed for 0.2 seconds see down below)
                    elevatedShooter.shooter.startShooting();
                }
                
                break;
            case SHOOT:
                elevatedShooter.shooter.startFeeding(1);
                elevatedShooter.shooter.startShooting();
                if (Util.getElaspedTime(startTime) > 1) {
                    state = State.DONE;
                }
            case DONE:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        elevatedShooter.shooter.stopFeeding();
        elevatedShooter.shooter.stopShooting();
        AdvantageScope.getInstance().dropAllNotes();
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putString("ShootSpeaker Status", state.name());
        // if (RobotBase.isSimulation()) return false;
        return state == State.DONE;
    }

    private double calculateAngle() {
        // everything in meters:
        Pose2d speakerPose = new Pose2d(0,5.6,new Rotation2d());
        double SPEAKER_HEIGHT = 2.1;


        double distance = Math.sqrt(
            Math.pow(robotDrive.getPose().getY() - speakerPose.getY(),2)
          + Math.pow(robotDrive.getPose().getX() - speakerPose.getX(), 2)
        );
        Util.consoleLog("distance %f", distance);

        double angle = -Math.toDegrees(Math.atan(SPEAKER_HEIGHT / distance)) - 30;
        angle = Util.clampValue(angle, -80, 0);

        Util.consoleLog("angle %f", angle);
        return angle;
    }
}
