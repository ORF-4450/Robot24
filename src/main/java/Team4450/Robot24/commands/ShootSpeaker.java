package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.AdvantageScope;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.Elevator;
import Team4450.Robot24.subsystems.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootSpeaker extends Command {
    private final DriveBase robotDrive;
    private final ElevatedShooter elevatedShooter;
    private double startTime;
    private  enum State {NONE, MOVING, BACKFEED, SHOOT, DONE};
    private State state = State.NONE;

    public ShootSpeaker(ElevatedShooter elevatedShooter, DriveBase robotDrive) {
        SmartDashboard.putString("ShootSpeaker Status", state.name());
        this.elevatedShooter = elevatedShooter;
        this.robotDrive = robotDrive;
        addRequirements(elevatedShooter);
    }
    @Override
    public void initialize() {
        elevatedShooter.shooter.enableClosedLoopFeedStop(false);
        state = State.BACKFEED;
        startTime = Util.timeStamp();
    }

    @Override
    public void execute() {
        SmartDashboard.putString("ShootSpeaker Status", state.name());
        // elevatedShooter.shooter.setAngle(calculateAngle());
        switch (state) {
            case NONE:
                break;
            case MOVING:
                break;
            case BACKFEED:
                if (Util.getElaspedTime(startTime) < 0.1) {
                    elevatedShooter.shooter.startFeeding(-0.3); // start by feeding the note backwards a bit (30% speed for 0.2 seconds see down below)
                    elevatedShooter.shooter.startShooting();
                }
                else if (Util.getElaspedTime(startTime) > 0.5) {
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
        return state == State.DONE;
    }

    private double calculateAngle() {
        Transform2d transform = robotDrive.getPose().minus(new Pose2d(0, 0, new Rotation2d()));

        // everything in meters:
        double distance = Math.sqrt(Math.pow(transform.getY(),2) + Math.pow(transform.getX(), 2));
        double SPEAKER_HEIGHT = 3;

        double angle = Math.toDegrees(Math.atan2(SPEAKER_HEIGHT, distance));

        Util.consoleLog("angle %f", angle);
        // shooter.setAngle(angle);
        // double wheelSpeed = 
        return angle;
    }
}
