package Team4450.Robot24.subsystems;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Robot24.RobotContainer;
import Team4450.Robot24.commands.Utility.NotifierCommand;

import static Team4450.Robot24.Constants.*;

import Team4450.Lib.FunctionTracer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class hosts functions relating to communicating with the ShuffleBoard driver
 * station application. Primarily, it's periodic function handles the regular update
 * of the "LCD" panel's display of robot status information when the robot is active.
 * This class supports running LCD updates in a separate thread to reduce overhead
 * on the main robot control thread.
 */
public class ShuffleBoard extends SubsystemBase
{
    public int                  currentTab, numberOfTabs = 2;

    private NotifierCommand     updateCommand;
    private Notifier            notifier;

	public ShuffleBoard()
	{
        // We use a NotifierCommand to run the DS update process in a separate thread
        // from the main thread. We set that command as the default command for this
        // subsystem so the scheduler starts the command. After start, the notifier
        // runs all the time updating the DS every 25ms which is slightly slower than
        // the main thread update period.
        updateCommand = new NotifierCommand(this::updateDS, .025, "SB", this);

        this.setDefaultCommand(updateCommand);

		Util.consoleLog("ShuffleBoard created!");
	}

	// This method will be called once per scheduler run on the scheduler (main) thread. Only
    // used if not running the updateDS with the notifier.
	@Override
	public void periodic()
    {
        //updateDS();
    }

    /**
     * Update the LCD tab of the Shuffleboard. Do not call if this class is running in it's
     * own thread.
     */
    public void updateDS()
	{    
        if (tracing) FunctionTracer.INSTANCE.enterFunction("ShuffleBoard.updateDS");
      
        Pose2d pose = RobotContainer.driveBase.getPose();
        
        // Lines 1 & 2 handled elsewhere.

        LCD.printLine(LCD_4, "pose x=%.1fm  y=%.1fm  deg=%.1f  yaw=%.1f", pose.getX(), 
                      pose.getY(), pose.getRotation().getDegrees(), RobotContainer.driveBase.getYaw());

        LCD.printLine(LCD_6, "uLX=%.2f  uLY=%.2f - uRX=%.2f  uRY=%.2f", 
                      RobotContainer.utilityController.getLeftX(),
                      RobotContainer.utilityController.getLeftY(), 
                      RobotContainer.utilityController.getRightX(),
                      RobotContainer.utilityController.getRightY());
                          
        if (tracing) FunctionTracer.INSTANCE.exitFunction("ShuffleBoard.updateDS");
    }

    /**
     * Reset the shuffleboard indicators to disabled states. Runs in
     * a separate thread.
     */
    public void resetLEDs()
    {
        // Notifier runs the reset function in a separate thread.
        notifier = new Notifier(this::resetLEDIndicators);
        notifier.startSingle(0);
    }

    /**
     * Reset the Shuffleboard indicators to diabled states. Runs on
     * main thread.
     */
    private void resetLEDIndicators()
    {
        Util.consoleLog();
        
        SmartDashboard.putBoolean("Disabled", true);
        SmartDashboard.putBoolean("Auto Mode", false);
        SmartDashboard.putBoolean("Teleop Mode", false);
        SmartDashboard.putBoolean("FMS", DriverStation.isFMSAttached());
        SmartDashboard.putBoolean("Overload", false);
        SmartDashboard.putNumber("AirPressure", 0);
        SmartDashboard.putBoolean("TargetLocked", false);
        SmartDashboard.putBoolean("Autonomous Active", false);
        SmartDashboard.putBoolean("Shooter", false);
        SmartDashboard.putBoolean("Alternate Drive", false);
        SmartDashboard.putBoolean("Tracking", false);
        SmartDashboard.putBoolean("Feeder", false);
        SmartDashboard.putBoolean("Tracking", false);
        SmartDashboard.putBoolean("Intake", false);

    }

    /**
     * Switch tab on shuffleboard display by rotating through the tabs.
     * @return The new tab index (0-based).
     */
    public int switchTab()
    {
        currentTab++;

        if (currentTab > (numberOfTabs - 1)) currentTab = 0;

        Util.consoleLog("%d", currentTab);

        Shuffleboard.selectTab(currentTab);

        return currentTab;
    }

    /**
     * Switch tab on shuffleboard display by tab name. Will create the tab if
     * it does not already exist.
     * @param tabName The name of the tab to select.
     * @return The selected tab object.
     */
    public ShuffleboardTab switchTab(String tabName)
    {
        Util.consoleLog("%s", tabName);

        return Shuffleboard.getTab(tabName);
    }
}
