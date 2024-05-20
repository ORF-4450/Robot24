package Team4450.Robot24.commands;


import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.ElevatedShooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Spins up the shooter wheels at a given angle and speed.
 */
public class SpinUpShooter extends Command {
    private final ElevatedShooter elevatedShooter;
    private double startTime;
    private  enum State {NONE, MOVING, BACKFEED, DONE};
    private State state = State.NONE;

    private double angle;
    private double speed;
    private boolean delay;

    /**
     * Spin up the shooter wheels at given angle and speed in preperation for a shot.
     * @apiNote The delay parameter affects whether the command ends immediately and keeps
     * wheels spinning (to pass off to another command), or if it waits to end until a shot.
     * @param elevatedShooter the ElevatedShooter subsystem
     * @param robotDrive the drivebase
     * @param angle the desired angle (Double.NaN is shoot in place)
     * @param speed the desired speed bounded [0, 1] (but it could be negative I guess -cole)
     * @param delay whether delay=true (end right away but keep spinning),
     * or =false (don't end until another command ends it)
     */
    public SpinUpShooter(ElevatedShooter elevatedShooter, DriveBase robotDrive, double angle, double speed, boolean delay) {
        SmartDashboard.putString("ShootSpeaker Status", state.name());
        this.elevatedShooter = elevatedShooter;
        this.angle = angle;
        this.speed = speed;
        this.delay = delay;
        addRequirements(elevatedShooter);
    }


    @Override
    public void initialize() {
        Util.consoleLog();
        elevatedShooter.shooter.hasShot = false; // this will be changed to end it by ShootSpeaker or similar
        elevatedShooter.shootDoesTheSpeakerInsteadOfTheAmp = true; // score button does Speaker rather than Amp
        
        if (Double.isNaN(angle)) { // if angle NaN just skip angle set and go straight to spinup
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
                double angleSetpoint = angle;
                // this is one of the few times where we specify exact state instead of using PresetPosition
                if (elevatedShooter.executeSetPosition(angleSetpoint, 0.15, false)) {
                    state = State.BACKFEED;
                    startTime = Util.timeStamp();
                }
                break;

            case BACKFEED: // run the sushis back a little so that it has more momentum feeding
                if (Util.getElaspedTime(startTime) < 0.06) {
                    elevatedShooter.shooter.startFeeding(-0.3);
                    // start by feeding the note backwards a bit (-30% speed for 0.05 seconds see above)
                }
                else {
                    elevatedShooter.shooter.startShooting(speed);
                    state = State.DONE;
                }
                break;

            case DONE:
                elevatedShooter.shooter.stopFeeding();
                elevatedShooter.shooter.startShooting(speed);
                SmartDashboard.putBoolean("Spun Up", true);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        elevatedShooter.shooter.stopFeeding();
        // if we aren't waiting for another command, cancel shot.
        // if we are waiting for another command, we trust it will handle cancel on its own
        if (!delay) elevatedShooter.shooter.stopShooting();
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putString("ShootSpeaker Status", state.name());
        
        if (delay)
            return state == State.DONE;
        else
            return elevatedShooter.shooter.hasShot; // wait for signal from other command
    }
}
