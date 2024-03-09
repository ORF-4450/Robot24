package Team4450.Robot24.commands;


import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.ElevatedShooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinUpShooter extends Command {
    private final ElevatedShooter elevatedShooter;
    private double startTime;
    private  enum State {NONE, MOVING, BACKFEED, DONE};
    private State state = State.NONE;

    private double angle;
    private boolean justShoot = false;

    public SpinUpShooter(ElevatedShooter elevatedShooter, DriveBase robotDrive, double angle) {
        SmartDashboard.putString("ShootSpeaker Status", state.name());
        this.elevatedShooter = elevatedShooter;
        this.angle = angle;
        addRequirements(elevatedShooter);
    }
    public SpinUpShooter(ElevatedShooter elevatedShooter, DriveBase robotDrive) {
        this(elevatedShooter, robotDrive, Double.NaN);
    }

    public SpinUpShooter(ElevatedShooter elevatedShooter, DriveBase robotDrive, boolean justShoot) {
        SmartDashboard.putString("ShootSpeaker Status", state.name());
        this.elevatedShooter = elevatedShooter;
        this.justShoot = justShoot;
        this.angle = 0;
        addRequirements(elevatedShooter);
    }

    @Override
    public void initialize() {
        elevatedShooter.shooter.hasShot = false;
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
                    angleSetpoint = 0;
                } else {
                    angleSetpoint = angle;
                }
                if (elevatedShooter.executeSetPosition(angleSetpoint, 0, 0.15, false)) {
                    state = State.BACKFEED;
                    startTime = Util.timeStamp();
                }
                break;
            case BACKFEED:
                if (Util.getElaspedTime(startTime) < 0.05) {
                    elevatedShooter.shooter.startFeeding(-0.3); // start by feeding the note backwards a bit (30% speed for 0.2 seconds see down below)
                    elevatedShooter.shooter.startShooting();
                }
                else {
                    state = State.DONE;
                }
                break;
            case DONE:
                elevatedShooter.shooter.stopFeeding();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        elevatedShooter.shooter.stopFeeding();
        elevatedShooter.shooter.stopShooting();
        SmartDashboard.putBoolean("Spun Up", true);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putString("ShootSpeaker Status", state.name());
        // return state == State.DONE;
        return elevatedShooter.shooter.hasShot;
    }
}
