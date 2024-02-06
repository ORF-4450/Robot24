
package Team4450.Robot24;

import static Team4450.Robot24.Constants.*;

import Team4450.Lib.*;
import Team4450.Robot24.wpilib.TimedRobot;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This is the top class or starting point for team robot code. This class acts
 * as an interface between out team code and the FIRST supplied (WpiLib) robot
 * control infrastructure. The WpiLib host automatically loads this class, and
 * calls the functions corresponding to each mode, as described in the
 * TimedRobot documentation. If you change the name of this class or the
 * containing package after creating this project, you must also update the
 * Main.java file in the project.
 */

public class Robot extends TimedRobot 
{
  private RobotContainer  robotContainer;

  /**
   * This function is called when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    try {
      robot = this;   // Stored in Constants.

      LCD.clearAll();
      LCD.printLine(LCD_1, "Mode: RobotInit");

      // Set up our custom logger.

      Util.CustomLogger.setup();

      // The wpilib classes that underlie this class generate a lot of warning
      // messages that flood the Riolog and make it almost unusable. The warnings 
      // are about our code in the robotPeriodic() function taking longer than .02 
      // sec to execute. It's very hard to stay under this limit. So...copied classes 
      // from the wpilib name space to inside this project and modified them to allow
      // us to control these warnings and log some of them to our log file. The warnings 
      // from IterativeRobotBase can be turned on/off and the timeout set. Any warnings 
      // from that class will go to our log file. The CommandScheduler also generates 
      // essentially the same warnings but copying that is getting beyond what we should
      // be doing, so we just set its internal timeout (because it allows us to) to a 
      // longer value to turn off its warnings. It does not log to our log file. These 
      // warnings can be turned back on at times to check if we are having significant 
      // overruns but turned off if things look ok. This is a major hack, the downside 
      // of which is that with each yearly release of Wpilib the copied files would have
      // to be recopied and remodified to make sure our copies are up to date. 
      // IterativeRobotBase and Watchdog have been modified. TimedRobot is needed to 
      // pull in these modified classes. Look for "4450" in the code for the mods.
      //
      // Note that the periodic function is called very .02 sec. If our code runs too
      // long that can lead to various control problems. But, it has proven hard to
      // do anything useful and not exceed the .02 sec watchdogs, though we have made
      // improvements to various functions to reduce execution time or used threading.
      // We have trimmed the volume of overrun messages but they still occur.

      enableWatchDogWarning(false);
      enableWatchDogFlush(false);
      this.setWatchDogTimeout(.04);
      CommandScheduler.getInstance().setPeriod(1.0);

      // Set Java to catch any uncaught exceptions and record them in our log file.

      Thread.setDefaultUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler() 
      {
        public void uncaughtException(Thread t, Throwable e) 
        {
          Util.consoleLog("Uncaught exception from thread " + t);
          Util.logException(e);
          robot.endCompetition();
        }
      });

      if (RobotBase.isSimulation()) Util.consoleLog("Simulated Robot");
      if (RobotBase.isReal()) Util.consoleLog("Real Robot");

      // Create SendableVersion object so it can be sent to the dashboard and also
      // log some of it's information.

      SendableVersion.INSTANCE.init(PROGRAM_NAME);

      // Note: under simulation, this information may not be not correct.
      Util.consoleLog("%s compiled by %s at %s (branch=%s, commit=%s)", SendableVersion.INSTANCE.getProgramVersion(),
          SendableVersion.INSTANCE.getUser(), SendableVersion.INSTANCE.getTime(), SendableVersion.INSTANCE.getBranch(),
          SendableVersion.INSTANCE.getCommit());

      //Util.consoleLog("manifest path=%s", SendableVersion.INSTANCE.getPath());

      // Send program version to the dashboard.
      SmartDashboard.putString("Program", PROGRAM_NAME);

      // Log RobotLib and WPILib versions we are using. Note Robolib WPILib version can be different
      // than robot WPILib version. Should be the same for best results.
      Util.consoleLog("Robot WPILib=%s  Java=%s", WPILibVersion.Version, System.getProperty("java.version"));
      Util.consoleLog("RobotLib=%s", LibraryVersion.version);

      // Note: Any Sendables added to SmartDashboard or Shuffleboard are sent to the DS on every
      // loop of a TimedRobot. In this case it means that the SendableVersion data would be sent
      // to the DS every 20ms even though it does not change. Sendables must be added to the SDB
      // or SB in order to be sent so its a catch-22 with static Sendables. So we add the SendableVersion
      // here and then a few lines below delete it from the sendable system. This puts the version
      // info onto the dashboard but removes it from further updates.

      // Note: As of 2023 WPILib, deleting a Sendable actually removes the data from the dashboard
      // so we had to replace adding the SendableVersion as a Sendable (putdata) and add the data
      // manually to the dashboard in SendableVersion class.

      SendableVersion.INSTANCE.updateDashBoard();

      // Instantiate our RobotContainer class. This will perform all necessary setup of the various
      // subsystems, commands and other items that are needed to to be ready before we start doing
      // either autonomous or teleop modes.

      robotContainer = new RobotContainer();
            
    } catch (Exception e) {
      Util.logException(e);
      endCompetition();
    }

    Util.consoleLog("end -------------------------------------------------------------------------");
  }

  /**
   * This function is called every driver station packet, no matter the mode. Use
   * this for items like diagnostics that you want run during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    // This function is called approx every .02 second.
    // Runs the Scheduler. It is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods. Scheduler must be called from the robot's periodic
    // function in order for anything in the Command-based framework to work.

    // WARNING: This function is called repeatedly even when robot is DISABLED. This means the
    // periodic method in all subsystems will be called even when disabled. The scheduler will
    // stop commands when disabled but not subsystems. It is also possible to set Commands to run
    // when robot is disabled. This seems a bad idea...

    // The try/catch will catch any exceptions thrown in the commands run by the
    // scheduler and record them in our log file then stops execution of this program.

    if (tracing & isEnabled()) FunctionTracer.INSTANCE.enterFunction("Robot.robotPeriodic");

    try {
      CommandScheduler.getInstance().run();
    } catch (Exception e) {
      Util.logException(e);
      this.endCompetition();
    }

		if (tracing & isEnabled()) FunctionTracer.INSTANCE.exitFunction("Robot.robotPeriodic");

    // if (tracing & isEnabled()) Tracer.INSTANCE.printFunctions(Util.logPrintStream);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    Util.consoleLog();

    LCD.printLine(LCD_1, "Mode: Disabled");

    // Reset driver station LEDs.

    RobotContainer.shuffleBoard.resetLEDs();

    // if (tracing)
    // {
    //   FunctionTracer.INSTANCE.printFunctions(Util.logPrintStream);
    // }

    Util.consoleLog("end -------------------------------------------------------------------------");
  }

  /**
   * This function is called periodically during disabled mode. Technically there
   * should be nothing here.
   */
  @Override
  public void disabledPeriodic() 
  {
  }

  /**
   * This function is called once at the start of autonomous mode and schedules
   * the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() 
  {
    Util.consoleLog("-------------------------------------------------------------------------");

    LCD.clearAll();

    LCD.printLine(LCD_1, "Mode: Auto - No Program");

    SmartDashboard.putBoolean("Disabled", false);
    SmartDashboard.putBoolean("Auto Mode", true);

    robotContainer.getMatchInformation();

    robotContainer.resetFaults();
    
    // RobotContainer function determines which auto command is selected to run.

    Command autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)

    try {
      if (autonomousCommand != null) autonomousCommand.schedule();
    } catch (Exception e) {
      Util.logException(e);
      this.endCompetition();
    }

    Util.consoleLog("end -------------------------------------------------------------------------");
  }

  /**
   * This function is called periodically during autonomous. Technically there
   * should be nothing here.
   */
  @Override
  public void autonomousPeriodic() 
  {
  }

  /**
   * This function is called once at the start of teleop mode.
   */
  @Override
  public void teleopInit() 
  {
    Util.consoleLog("-------------------------------------------------------------------------");

    robotContainer.getMatchInformation();

    LCD.clearAll();

    LCD.printLine(LCD_1, "Mode: Teleop  All=%s, Start=%d, FMS=%b, msg=%s", alliance.name(), location,
                  DriverStation.isFMSAttached(), gameMessage);

    SmartDashboard.putBoolean("Disabled", false);
    SmartDashboard.putBoolean("Teleop Mode", true);

    robotContainer.resetFaults();

    // Driving handled by DriveCommand which is default command for the DriveBase.
    // Other commands scheduled by joystick buttons.

    if  (tracing) FunctionTracer.INSTANCE.reset();

    Util.consoleLog("end -------------------------------------------------------------------------");
  }

  /**
   * This function is called periodically during teleop. Technically there should
   * be nothing here.
   */
  @Override
  public void teleopPeriodic() 
  {
    if (tracing) FunctionTracer.INSTANCE.printFunctions(Util.logPrintStream);
  }

  /**
   * This function is called once at the start of test mode.
   */
  @Override
  public void testInit() 
  {
    Util.consoleLog();

    // Cancels all running commands at the start of test mode.

    CommandScheduler.getInstance().cancelAll();

    // Next two lines launch teleop mode, but since we are in test
    // mode, LiveWindow will be enabled to display test data to the
    // outlineviewer to shuffleboard. Our "test" mode is the regular
    // telop with LW enabled.

    LiveWindow.enableAllTelemetry();

    teleopInit();

    CommandScheduler.getInstance().enable();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() 
  {
  }
}
