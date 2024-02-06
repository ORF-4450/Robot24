package Team4450.Robot24.commands.Utility;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A command that starts a notifier to run the given runnable periodically in a separate thread.
 * Has no end condition if period != 0; either subclass it or use {@link Command#withTimeout(double)} or
 * {@link Command#withInterrupt(BooleanSupplier)} to give it one. Modified by Team 4450 so that
 * Period == 0 runs runnable one time then command ends.
 *
 * <p>WARNING: Do not use this class unless you are confident in your ability to make the executed
 * code thread-safe.  If you do not know what "thread-safe" means, that is a good sign that
 * you should not use this class.
 */
public class NotifierCommand extends Command 
{
  protected final Notifier  m_notifier;
  protected final double 	  m_period;
  protected String          m_name = "";
  private   boolean         m_runWhenDisabled;

  /**
   * Creates a new NotifierCommand. Caller must schedule the returned command for
   * it to execute.
   *
   * @param toRun        The runnable for the notifier to run.
   * @param period       The period at which the notifier should run, in seconds. Zero 
   *                     runs the runnable one time then the command ends.
   * @param requirements The subsystems required by this command.
   */
  public NotifierCommand(Runnable toRun, double period, Subsystem... requirements) 
  {
    Util.consoleLog();

	  m_notifier = new Notifier(toRun);
	  m_period = period;
	  addRequirements(requirements);
  }

  /**
   * Creates a new NotifierCommand. Caller must schedule the returned command for
   * it to execute.
   *
   * @param toRun        The runnable for the notifier to run.
   * @param period       The period at which the notifier should run, in seconds. Zero 
   *                     runs the runnable one time then the command ends.
   * @param name         Name to identify the thread.
   * @param requirements The subsystems required by this command.
   */
  public NotifierCommand(Runnable toRun, double period, String name, Subsystem... requirements) 
  {
    this(toRun, period, requirements);
    m_name = "Notifier-" + name;
    m_notifier.setName(m_name);

    Util.consoleLog("%s created!", m_name);
  }

  /**
   * Creates a new NotifierCommand. Caller must schedule the command for
   * it to execute.
   *
   * @param toRun        The runnable for the notifier to run.
   * @param period       The period at which the notifier should run, in seconds. Zero 
   *                     runs the runnable one time then the command ends.
   */
  public NotifierCommand(Runnable toRun, double period) 
  {
    Util.consoleLog();

	  m_notifier = new Notifier(toRun);
	  m_period = period;
  }

  /**
   * Creates a new NotifierCommand. Caller must schedule the returned command for
   * it to execute.
   *
   * @param toRun        The runnable for the notifier to run.
   * @param period       The period at which the notifier should run, in seconds. Zero 
   *                     runs the runnable one time then the command ends.
   * @param name         Name for notifier thread.
   */
  public NotifierCommand(Runnable toRun, double period, String name) 
  {
    this(toRun, period);
    m_name = "Notifier-" + name;
    m_notifier.setName(m_name);

    Util.consoleLog("Notifier-%s created!", m_name);
  }

  @Override
  public void initialize() 
  {
    Util.consoleLog("%s period=%.3f", m_name, m_period);

	  if (m_period == 0)
		  m_notifier.startSingle(0);
	  else
		  m_notifier.startPeriodic(m_period);
  }

  @Override
  public void end(boolean interrupted) 
  {
    Util.consoleLog("%s interrupted=%b", m_name, interrupted);

    m_notifier.stop();

    //Util.consoleLog("%s --end--", m_name);
  }
	
  @Override
  public boolean isFinished()
  {
    //Util.consoleLog(m_name);

	  if (m_period == 0)
		  return true;
	  else
		  return false;
  }

  /**
   * Returns state of run when disabled setting.
   * @return True if command runs when disabled.
   */
  @Override
  public boolean runsWhenDisabled()
  {
      return m_runWhenDisabled;
  }

  /**
   * Sets command to run when robot disabled. Defaults to false.
   * @param run True to run when disabled, false if not.
   */
  public void setRunWhenDisabled(boolean run)
  {
      m_runWhenDisabled = run;
  }
}
