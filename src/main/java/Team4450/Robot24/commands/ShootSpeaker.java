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
import edu.wpi.first.wpilibj2.command.Command;

public class ShootSpeaker extends Command {
    private final DriveBase robotDrive;
    private final ElevatedShooter elevatedShooter;
    private double startTime;

    public ShootSpeaker(ElevatedShooter elevatedShooter, DriveBase robotDrive) {
        this.elevatedShooter = elevatedShooter;
        this.robotDrive = robotDrive;
        addRequirements(elevatedShooter);
    }
    @Override
    public void initialize() {
        elevatedShooter.shooter.enableClosedLoopFeedStop(false);
        // start by feeding the note backwards a bit (0.2 seconds)
        elevatedShooter.shooter.startFeeding(-0.3);
        startTime = Util.timeStamp();
    }

    @Override
    public void execute() {
        if (Util.getElaspedTime(startTime) > 0.2) {
            elevatedShooter.shooter.startShooting();
            elevatedShooter.shooter.startFeeding(1);
        }
        // calculateAngle();
        // commented because handled in end():
        if (!elevatedShooter.shooter.hasNote()){//(Util.getElaspedTime(startTime) > 1.2) {
            elevatedShooter.shooter.stopFeeding();
            elevatedShooter.shooter.stopShooting();
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
        return !elevatedShooter.shooter.hasNote();
    }

    // private double calculateAngle() {
    //     Transform2d transform = robotDrive.getPose().minus(new Pose2d(0, 0, new Rotation2d()));

    //     // everything in meters:
    //     double distance = Math.sqrt(Math.pow(transform.getY(),2) + Math.pow(transform.getX(), 2));
    //     double SPEAKER_HEIGHT = 3 - elevator.getHeight();

    //     double angle = Math.toDegrees(Math.atan2(SPEAKER_HEIGHT, distance));

    //     Util.consoleLog("%f", angle);
    //     shooter.setAngle(angle);
    //     // double wheelSpeed = 
    //     return 0; //TODO
    // }
}
