
package Team4450.Robot24;

import static Team4450.Robot24.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import Team4450.Lib.CameraFeed;
import Team4450.Lib.XboxController;
import Team4450.Robot24.commands.AimSpeaker;
import Team4450.Robot24.commands.Preset;
import Team4450.Robot24.commands.DriveCommand;
import Team4450.Robot24.commands.DriveToNote;
import Team4450.Robot24.commands.IntakeNote;
import Team4450.Robot24.commands.PointToYaw;
import Team4450.Robot24.commands.ReverseIntake;
import Team4450.Robot24.commands.ShootAmp;
import Team4450.Robot24.commands.ShootSpeaker;
import Team4450.Robot24.commands.SpinUpShooter;
import Team4450.Robot24.commands.UpdateCandle;
import Team4450.Robot24.commands.UpdateVisionPose;
import Team4450.Robot24.commands.AimSpeaker.Position;
import Team4450.Robot24.subsystems.Candle;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.ElevatedShooter;
import Team4450.Robot24.subsystems.PhotonVision;
import Team4450.Robot24.subsystems.Intake;
import Team4450.Robot24.subsystems.ShuffleBoard;
import Team4450.Robot24.subsystems.ElevatedShooter.PresetPosition;
import Team4450.Robot24.subsystems.PhotonVision.PipelineType;
import Team4450.Lib.MonitorPDP;
import Team4450.Lib.NavX;
import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
	// Subsystems.

	public static ShuffleBoard	shuffleBoard;
	public static DriveBase 	driveBase;
	// public static PhotonVision	pvFrontCamera;
	public static PhotonVision	pvShooterCamera;
	public static PhotonVision	pvNoteCamera;
	private final Intake       	intake;
	private final ElevatedShooter elevShooter;
	private final Candle        candle;
	
	// Subsystem Default Commands.

    // Persistent Commands.

	// Some notes about Commands.
	// When a Command is created with the New operator, its constructor is called. When the
	// command is added to the Scheduler to be run, its initialize method is called. Then on
	// each scheduler run, as long as the command is still scheduled, its execute method is
	// called followed by isFinished. If isFinished it false, the command remains in the
	// scheduler list and on next run, execute is called followed by isFinihsed. If isFinished
	// returns true, the end method is called and the command is removed from the scheduler list.
	// Now if you create another instance with new, you get the constructor again. But if you 
	// are re-scheduling an existing command instance (like the ones above), you do not get the
	// constructor called, but you do get initialize called again and then on to execute & etc.
	// So this means you have to be careful about command initialization activities as a persistent
	// command in effect has two lifetimes (or scopes): Class global and each new time the command
	// is scheduled. Note the FIRST doc on the scheduler process is not accurate as of 2020.
	
	// GamePads. 2 Game Pads use RobotLib XboxController wrapper class for some extra features.
	// Note that button responsiveness may be slowed as the schedulers command list gets longer 
	// or commands get longer as buttons are processed once per scheduler run.
	
	private XboxController			driverController =  new XboxController(DRIVER_PAD);
	public static XboxController	utilityController = new XboxController(UTILITY_PAD);

	//private AnalogInput			pressureSensor = new AnalogInput(PRESSURE_SENSOR);
	  
	// private PowerDistribution	pdp = new PowerDistribution(REV_PDB, PowerDistribution.ModuleType.kCTRE);
	private PowerDistribution	pdp = new PowerDistribution(REV_PDB, PowerDistribution.ModuleType.kRev);

	// PneumaticsControlModule class controls the PCM. New for 2022.
	//private PneumaticsControlModule	pcm = new PneumaticsControlModule(COMPRESSOR);

	// Navigation board.
	public static NavX			navx;

	private MonitorPDP     		monitorPDPThread;
	//private MonitorCompressor	monitorCompressorThread;
    private CameraFeed			cameraFeed;
    
	// Trajectories we load manually.
	//public static PathPlannerTrajectory	ppTestTrajectory;

	private static SendableChooser<Command>	autoChooser;
	
	private static String 					autonomousCommandName = "none";

	/**
	 * The container for the robot. Contains subsystems, Opertor Interface devices, and commands.
	 */
	public RobotContainer() throws Exception
	{
		Util.consoleLog();

		SmartDashboard.putData("PDH", pdp);
		// Use below for production to reduce NT traffic.           
	    //SendableRegistry.addLW(pdp, "PDH");

		// Get information about the match environment from the Field Control System.
      
		getMatchInformation();

		// Read properties file from RoboRio "disk". If we fail to open the file,
		// log the exception but continue and default to competition robot.
      
		try {
			robotProperties = Util.readProperties();
		} catch (Exception e) { Util.logException(e);}

		// Is this the competition or clone robot?
   		
		if (robotProperties == null || robotProperties.getProperty("RobotId").equals("comp"))
			isComp = true;
		else
			isClone = true;
 		
		// Set compressor enabled switch on dashboard from properties file.
		// Later code will read that setting from the dashboard and turn 
		// compressor on or off in response to dashboard setting.
 		
		boolean compressorEnabled = true;	// Default if no property.

		if (robotProperties != null) 
			compressorEnabled = Boolean.parseBoolean(robotProperties.getProperty("CompressorEnabledByDefault"));
		
		SmartDashboard.putBoolean("CompressorEnabled", compressorEnabled);

		// Reset PDB & PCM sticky faults.
    
		resetFaults();

		// Create NavX object here since must done before CameraFeed is created (don't remember why).
        // Navx calibrates at power on and must complete before robot moves. Takes ~1 second for 2nd
        // generation Navx ~15 seconds for classic Navx. We assume there will be enough time between
        // power on and our first movement because normally things don't happen that fast

		// Warning: The navx instance is shared with the swerve drive code. Resetting or otherwise
		// manipulating the navx (as opposed to just reading data) may crash the swerve drive code.

		navx = NavX.getInstance(NavX.PortType.SPI);

		// Add navx as a Sendable. Updates the dashboard heading indicator automatically.
 		
		SmartDashboard.putData("Gyro2", navx);

		// Invert driving joy sticks Y axis so + values mean forward.
		// Invert driving joy sticks X axis so + values mean right.
	  
		driverController.invertY(true);
		driverController.invertX(true);		

		// Create subsystems prior to button mapping.

		shuffleBoard = new ShuffleBoard();
		driveBase = new DriveBase();
		pvShooterCamera = new PhotonVision(CAMERA_SHOOTER, PipelineType.POSE_ESTIMATION, CAMERA_SHOOTER_TRANSFORM);
		pvNoteCamera = new PhotonVision(CAMERA_NOTE, PipelineType.OBJECT_TRACKING, CAMERA_NOTE_TRANSFORM);
		intake = new Intake();
		elevShooter = new ElevatedShooter();
		candle = new Candle(CTRE_CANDLE, 8+26);

		// Create any persistent commands.

		// Set any subsystem Default commands.

		// This sets up the photonVision subsystem to constantly update the robotDrive odometry
	    // with AprilTags (if it sees them). (As well as vision simulator)
    	// pvFrontCamera.setDefaultCommand(new UpdateVisionPose(pvFrontCamera, driveBase));
		pvNoteCamera.setDefaultCommand(new UpdateVisionPose(pvNoteCamera, driveBase));
		pvShooterCamera.setDefaultCommand(new UpdateVisionPose(pvShooterCamera, driveBase));

		candle.setDefaultCommand(new UpdateCandle(candle));

		// Set the default drive command. This command will be scheduled automatically to run
		// every teleop period and so use the gamepad joy sticks to drive the robot. 

		// The joystick controls for driving:
		// Left stick Y axis -> forward and backwards movement (throttle)
		// Left stick X axis -> left and right movement (strafe)
		// Right stick X axis -> rotation
		// Note: X and Y axis on stick is opposite X and Y axis on the WheelSpeeds object
		// and the odometry pose2d classes.
		// Wheelspeeds +X axis is down the field away from alliance wall. +Y axis is left
		// when standing at alliance wall looking down the field.
		// This is handled here by swapping the inputs. Note that first axis parameter below
		// is the X wheelspeeds input and the second is Y wheelspeeds input.

		// Note that field oriented driving does the movements in relation to the field. So
		// throttle is always down the field and back and strafe is always left right from
		// the down the field axis, no matter which way the robot is pointing. Robot oriented
		// driving movemments are in relation to the direction the robot is currently pointing.

		driveBase.setDefaultCommand(new DriveCommand(driveBase,
		 							driverController.getLeftYDS(),
									driverController.getLeftXDS(), 
									driverController.getRightXDS(),
									driverController));

		// up and down on left operator controller joystick pivots shooter assembly
		// up and down on right operator controller joystick moves elevator assembly

		// Changed operator stick controls for Girls Gen 2024. -Arnav
		/* elevShooter.setDefaultCommand(new RunCommand(
			()->{elevShooter.moveRelative(
				-MathUtil.applyDeadband(utilityController.getLeftY(), DRIVE_DEADBAND), // pivot
				-MathUtil.applyDeadband(utilityController.getRightY(), DRIVE_DEADBAND)); // elevator
			},
			elevShooter, intake)); */
		
		// intake.setDefaultCommand(new ReverseIntake(intake));
		
		// new RunCommand(
		// 	() -> driveBase.drive(
		// 		-MathUtil.applyDeadband(driverController.getLeftY(), DRIVE_DEADBAND),
		// 		-MathUtil.applyDeadband(driverController.getLeftX(), DRIVE_DEADBAND),
		// 		-MathUtil.applyDeadband(driverController.getRightX(), DRIVE_DEADBAND),
		// 		false),
		// 	driveBase));

		// Start the compressor, PDP and camera feed monitoring Tasks.

   		// monitorCompressorThread = MonitorCompressor.getInstance(pressureSensor);
   		// monitorCompressorThread.setDelay(1.0);
   		// monitorCompressorThread.SetLowPressureAlarm(50);
   		// monitorCompressorThread.start();
		
   		monitorPDPThread = MonitorPDP.getInstance(pdp);
   		monitorPDPThread.start();
		
		// Start camera server thread using our class for usb cameras.
    
		cameraFeed = CameraFeed.getInstance(); 
		cameraFeed.start();
 		
		// Log info about NavX.
	  
		navx.dumpValuesToNetworkTables();
 		
		if (navx.isConnected())
			Util.consoleLog("NavX version=%s", navx.getAHRS().getFirmwareVersion());
		else
		{
			Exception e = new Exception("NavX is NOT connected!");
			Util.logException(e);
		}
        
        // Configure autonomous routines and send to dashboard.
		
		setAutoChoices();

		// Configure the button bindings.
		
        configureButtonBindings();
        
        // Load any trajectory files in a separate thread on first scheduler run.
        // We do this because trajectory loads can take up to 10 seconds to load so we want this
        // being done while we are getting started up. Hopefully will complete before we are ready to
        // use the trajectory.
		
		// NotifierCommand loadTrajectory = new NotifierCommand(this::loadTestTrajectory, 0);
        // loadTrajectory.setRunWhenDisabled(true);
        // CommandScheduler.getInstance().schedule(loadTrajectory);
		
		// //testTrajectory = loadTrajectoryFile("Slalom-1.wpilib.json");
		
		// loadTrajectory = new NotifierCommand(this::loadPPTestTrajectory, 0);
        // loadTrajectory.setRunWhenDisabled(true);
        // CommandScheduler.getInstance().schedule(loadTrajectory);

		//PathPlannerTrajectory ppTestTrajectory = loadPPTrajectoryFile("richard");

		Util.consoleLog("End robot container------------------------------");
	}

	/**
	 * Use this method to define your button->command mappings.
     * 
     * These buttons are for real robot driver station with 3 sticks and launchpad.
	 * The launchpad makes the colored buttons look like a joystick.
	 */
	private void configureButtonBindings() 
	{
		Util.consoleLog();
	  
		// ------- Driver pad buttons -------------
		
		// For simple functions, instead of creating commands, we can call convenience functions on
		// the target subsystem from an InstantCommand. It can be tricky deciding what functions
		// should be an aspect of the subsystem and what functions should be in Commands...

		// POV buttons do same as alternate driving mode but without any lateral
		// movement and increments of 45deg.
		// new Trigger(()-> driverController.getPOV() != -1)
		// 	.onTrue(new PointToYaw(()->PointToYaw.yawFromPOV(driverController.getPOV()), driveBase, false))

		// vibrate between 30 and 25 sec left in match.
		new Trigger(() -> Timer.getMatchTime() < 30 && Timer.getMatchTime() > 25).whileTrue(new StartEndCommand(
			() -> {
				driverController.setRumble(RumbleType.kBothRumble, 0.5);
				utilityController.setRumble(RumbleType.kBothRumble, 0.5);},
			() -> {
				driverController.setRumble(RumbleType.kBothRumble, 0);
				utilityController.setRumble(RumbleType.kBothRumble, 0);
		}));

		// vibrate on note pickup.
		new Trigger(() -> elevShooter.shooter.hasNote()).onTrue(new InstantCommand(()->{
			driverController.setRumble(RumbleType.kBothRumble, 0.5);
			}).andThen(new WaitCommand(0.5)).andThen(new InstantCommand(()->{
			driverController.setRumble(RumbleType.kBothRumble, 0);
		})));

		// holding top right bumper enables the alternate rotation mode in
		// which the driver points stick to desired heading.
		new Trigger(() -> driverController.getRightBumper())
			.whileTrue(new PointToYaw(
				()->PointToYaw.yawFromAxes(
					-MathUtil.applyDeadband(driverController.getRightX(), Constants.DRIVE_DEADBAND),
					-MathUtil.applyDeadband(driverController.getRightY(), Constants.DRIVE_DEADBAND)
				), driveBase, false
		));

		// toggle slow-mode
		new Trigger(() -> driverController.getLeftBumper())
			.whileTrue(new StartEndCommand(driveBase::enableSlowMode, driveBase::disableSlowMode));

		// aim trap 		SmartDashboard.putNumber("trap/speed", 0.7);
		// .whileTrue(
		// 		new Preset(elevShooter, PresetPosition.TRAP).andThen(
		// 		new SpinUpShooter(elevShooter, driveBase, Double.NaN, SmartDashboard.getNumber("trap/speed", 0.7), false)
		// 	).alongWith(new AimTrap(driveBase, pvShooterCamera)
		// ));

		// reset field orientation
		new Trigger(() -> driverController.getStartButton())
			.onTrue(new InstantCommand(driveBase::zeroGyro));

		// toggle field-oriented
		new Trigger(() -> driverController.getBackButton())
			.onTrue(new InstantCommand(driveBase::toggleFieldRelative));

		// Holding x button sets X pattern to stop movement.
		new Trigger(() -> driverController.getXButton())
			.whileTrue(new RunCommand(() -> driveBase.setX(), driveBase));

		// toggle brake mode
		new Trigger(() -> driverController.getAButton())
    		.onTrue(new InstantCommand(driveBase::toggleBrakeMode));
		
		// hold face note
		new Trigger(() -> driverController.getRightTrigger())// && !elevShooter.hasNote())
    	    .whileTrue(new DriveToNote(driveBase, pvNoteCamera, false));
		
		new Trigger(() -> driverController.getLeftTrigger())
			.whileTrue(new DriveToNote(driveBase, pvNoteCamera, false));
		
		// new Trigger(() -> driverController.getRightTrigger())// && elevShooter.hasNote())
    	//     .whileTrue(new FaceAprilTag(driveBase, pvShooterCamera));
		
		// -------- Utility pad buttons ----------
		
		// climb position
		new Trigger(()-> utilityController.getPOV() == 0) // up POV
			.toggleOnTrue(new Preset(elevShooter, PresetPosition.CLIMB));
		
		// shoot amp
		new Trigger(()-> utilityController.getPOV() == 90) // right POV
			.toggleOnTrue(new ParallelCommandGroup(
				new Preset(elevShooter, PresetPosition.SHOOT_AMP_FRONT),
				new InstantCommand(()->elevShooter.shootDoesTheSpeakerInsteadOfTheAmp = false)
			));
		
		// shoot amp (same as above just the other button)
		new Trigger(()-> utilityController.getPOV() == 270) // left POV
			.toggleOnTrue(new ParallelCommandGroup(
				new Preset(elevShooter, PresetPosition.SHOOT_AMP_FRONT),
				new InstantCommand(()->elevShooter.shootDoesTheSpeakerInsteadOfTheAmp = false)
			));

		// DRIVER emergency overrides for preset positions: never used
		new Trigger(() -> driverController.getPOV() == 0).toggleOnTrue( // podium
			new Preset(elevShooter, PresetPosition.HIGH_SHOT).andThen(
			new SpinUpShooter(elevShooter, driveBase, -39, 1, false)));
		
		new Trigger(() -> driverController.getPOV() == 180).toggleOnTrue( // amp
			new Preset(elevShooter, PresetPosition.HIGH_SHOT).andThen(
			new SpinUpShooter(elevShooter, driveBase, -39, 1, false)));

		// Climb functionality
		new Trigger(()-> utilityController.getPOV() == 180) // down POV
			.toggleOnTrue(
				(new Preset(elevShooter, PresetPosition.CLIMB_2)).andThen(
					new Preset(elevShooter, PresetPosition.CLIMB_3)
				)
			);
		
		// shoot/score button if in speaker shot mode
		new Trigger(() -> utilityController.getRightBumper() && elevShooter.shootDoesTheSpeakerInsteadOfTheAmp)
			.toggleOnTrue(new ShootSpeaker(elevShooter));
		
		// shoot/score button if in amp shot mode
		new Trigger(() -> utilityController.getRightBumper() && !elevShooter.shootDoesTheSpeakerInsteadOfTheAmp)
			.toggleOnTrue(new ShootAmp(elevShooter));

		// start intake
		new Trigger(() -> utilityController.getLeftBumper())
			.toggleOnTrue(
				new IntakeNote(intake, elevShooter));
				//.andThen(
			// 	new SpinUpShooter(elevShooter, driveBase, true).andThen(
			// 	new AimSpeaker(driveBase, elevShooter, pvShooterCamera, pvFrontCamera, driverController.getRightXDS())
			// )));

		// hold manual feed/intake
		new Trigger(()->utilityController.getLeftTrigger()).whileTrue(new StartEndCommand(
			()->{elevShooter.shooter.startFeeding(1);elevShooter.shooter.backShoot();intake.start();},
			()->{elevShooter.shooter.stopFeeding();elevShooter.shooter.stopShooting();intake.stop();}
		));

		// hold manual outtake
		new Trigger(()->utilityController.getRightTrigger()).whileTrue(new StartEndCommand(
			()->{elevShooter.shooter.startFeeding(-1);elevShooter.shooter.backShoot();intake.start(-0.9);},
			()->{elevShooter.shooter.stopFeeding();elevShooter.shooter.stopShooting();intake.stop();}
		));

		// outtake: doesn't really work use manual hold
		new Trigger(() -> utilityController.getBackButton())
			.toggleOnTrue(new ReverseIntake(intake, elevShooter));

		// lob shot
		new Trigger(() -> utilityController.getStartButton())
			.onTrue(new SpinUpShooter(elevShooter, driveBase, -45, 0.65, false));
		
		// high shot vision
		new Trigger(() -> utilityController.getYButton()) // HIGH VISION TRACKING
			.toggleOnTrue(
				new Preset(elevShooter, PresetPosition.HIGH_SHOT).andThen(//-20
				new SpinUpShooter(elevShooter, driveBase, Double.NaN, 1, true).andThen(
				new AimSpeaker(driveBase, elevShooter, pvShooterCamera, driverController.getRightXDS(), Position.HIGH)
			)));
		
		// spin up in place
		new Trigger(() -> utilityController.getXButton()) // manual
			.toggleOnTrue(new SpinUpShooter(elevShooter, driveBase, Double.NaN, 1, false));

		// subwoofer preset
		new Trigger(() -> utilityController.getAButton()) // SUBWOOFER
			.toggleOnTrue(new SpinUpShooter(elevShooter, driveBase, SUBWOOFER_ANGLE, 1, false));
		
		// new Trigger(() -> utilityController.getBButton()) // OUTER RING
		// 	.toggleOnTrue(new SpinUpShooter(elevShooter, driveBase, OUTER_ANGLE));

		// vision shot
		new Trigger(() -> utilityController.getBButton()) // VISION TRACKING
			.toggleOnTrue(
				new Preset(elevShooter, PresetPosition.SHOOT_VISION_START).andThen(
				new SpinUpShooter(elevShooter, driveBase, -39, 1, true).andThen(
				new AimSpeaker(driveBase, elevShooter, pvShooterCamera, driverController.getRightXDS(), Position.NORMAL)
			)));
		
	}

	/**
	 * Use this to pass the autonomous command(s) to the main {@link Robot} class.
	 * Determines which auto command from the selection made by the operator on the
	 * DS drop down list of commands.
	 * @return The Command to run in autonomous.
	 */
	public Command getAutonomousCommand() {
		// PathPlannerAuto  	ppAutoCommand;
		Command				autoCommand;

		autoCommand = autoChooser.getSelected();

		if (autoCommand == null) 
		{
			autonomousCommandName = "none";

			return autoCommand;
		}

		autonomousCommandName = autoCommand.getName();

		Util.consoleLog("auto name=%s", autonomousCommandName);

		if (autoCommand instanceof PathPlannerAuto)
		{
			// ppAutoCommand = (PathPlannerAuto) autoCommand;
	
			// Util.consoleLog("pp starting pose=%s", PathPlannerAuto.getStaringPoseFromAutoFile(autoCommand.getName().toString()));
		}

		return autoCommand;
  	}

	public static String getAutonomousCommandName()
	{
		return autonomousCommandName;
	}
  
    // Configure SendableChooser (drop down list on dashboard) with auto program choices and
	// send them to SmartDashboard/ShuffleBoard.
	
	private void setAutoChoices()
	{
	 	Util.consoleLog();
		
		// Register commands called from PathPlanner Autos.

		NamedCommands.registerCommand("IntakeNoteVision", new ParallelDeadlineGroup(
			new IntakeNote(intake, elevShooter).withTimeout(1),
			new DriveToNote(driveBase, pvNoteCamera, true)
		));

		NamedCommands.registerCommand("IntakeNote", new IntakeNote(intake, elevShooter));
		
		NamedCommands.registerCommand("IntakeNoteShooting", new StartEndCommand(()->{
			intake.start();
			elevShooter.shooter.startShooting();
			elevShooter.shooter.startFeeding(1);
		}, () -> {
			intake.stop();
			elevShooter.shooter.stopFeeding();
			elevShooter.shooter.stopShooting();
		}, intake));

		NamedCommands.registerCommand("DropNote",
			new SpinUpShooter(elevShooter, driveBase, 0, 0.1, true).andThen(
			new WaitCommand(0).andThen(new ShootSpeaker(elevShooter))
		));
		
		NamedCommands.registerCommand("ShootSpeaker",
			new SpinUpShooter(elevShooter, driveBase, SUBWOOFER_ANGLE, 1, true).andThen(
			new WaitCommand(0.3).andThen(new ShootSpeaker(elevShooter))
		));
		
		NamedCommands.registerCommand("ShootPodium",
			new SpinUpShooter(elevShooter, driveBase, PODIUM_ANGLE, 1, true).andThen(
			new WaitCommand(0.8).andThen(new ShootSpeaker(elevShooter))
		));

		NamedCommands.registerCommand("SpinUpCenterline",new SpinUpShooter(elevShooter, driveBase, -28.5, 1, true));
		
		NamedCommands.registerCommand("ShootWing",
			new Preset(elevShooter, PresetPosition.WING_SHOT).andThen(
			new SpinUpShooter(elevShooter, driveBase, Double.NaN, 1, true).andThen(
			new WaitCommand(0.8).andThen(new ShootSpeaker(elevShooter))
		)));
		
		NamedCommands.registerCommand("ShootVision",
			new Preset(elevShooter, PresetPosition.SHOOT_VISION_START).andThen(
			new SpinUpShooter(elevShooter, driveBase, -39, 1, true).andThen(
			new AimSpeaker(driveBase, elevShooter, pvShooterCamera, driverController.getRightXDS(), Position.NORMAL))).alongWith(
			// AimSpeaker runs in paralell with the next two commands. 1 sec allows for aiming thhen shoot happens.
			new WaitCommand(1).andThen(new ShootSpeaker(elevShooter))
		));
		
		NamedCommands.registerCommand("ShootFar",
			new SpinUpShooter(elevShooter, driveBase, OUTER_ANGLE, 1, true).andThen(
			new WaitCommand(0.8).andThen(new ShootSpeaker(elevShooter))
		));
		
		NamedCommands.registerCommand("SpinUp",new SpinUpShooter(elevShooter, driveBase, Double.NaN, 1, false));
		NamedCommands.registerCommand("SpinUpSpeaker",new SpinUpShooter(elevShooter, driveBase, SUBWOOFER_ANGLE, 1, true));
		NamedCommands.registerCommand("SpinUpPodium",new SpinUpShooter(elevShooter, driveBase, PODIUM_ANGLE, 1, true));
		NamedCommands.registerCommand("SpinUpFar",new SpinUpShooter(elevShooter, driveBase, OUTER_ANGLE, 1, true));
		NamedCommands.registerCommand("StartShooterWheels",new InstantCommand(()->{elevShooter.shooter.startShooting();}));
		NamedCommands.registerCommand("WingPreset", new Preset(elevShooter, PresetPosition.WING_SHOT));
		NamedCommands.registerCommand("Shoot", new ShootSpeaker(elevShooter));

		NamedCommands.registerCommand("AimSpeaker", 
				new Preset(elevShooter, PresetPosition.SHOOT_VISION_START).andThen(
				new SpinUpShooter(elevShooter, driveBase, -39, 1, true).andThen(
				new AimSpeaker(driveBase, elevShooter, pvShooterCamera, driverController.getRightXDS(), Position.NORMAL)
		)));
		
		NamedCommands.registerCommand("AmpReady", new ParallelCommandGroup(
			new Preset(elevShooter, PresetPosition.SHOOT_AMP_FRONT),
			new InstantCommand(()->elevShooter.shootDoesTheSpeakerInsteadOfTheAmp = false)
		));
		
		NamedCommands.registerCommand("ShootAmp", new ShootAmp(elevShooter));

		// Create a chooser with the PathPlanner Autos located in the PP
		// folders.

	    autoChooser = AutoBuilder.buildAutoChooser();
		
    	SmartDashboard.putData("Auto Program", autoChooser);
	}

	/**
	 *  Get and log information about the current match from the FMS or DS.
	 */
	public void getMatchInformation()
	{
		alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  	  	location = DriverStation.getLocation().orElse(0);
  	  	eventName = DriverStation.getEventName();
	  	matchNumber = DriverStation.getMatchNumber();
	  	gameMessage = DriverStation.getGameSpecificMessage();
    
	  	Util.consoleLog("Alliance=%s, Location=%d, FMS=%b event=%s match=%d msg=%s", 
    		  		   alliance.name(), location, DriverStation.isFMSAttached(), eventName, matchNumber, 
    		  		   gameMessage);
	}
		
	/**
	 * Reset sticky faults in PDP and PCM and turn compressor on/off as
	 * set by switch on DS.
	 */
	public void resetFaults()
	{
		// This code turns on/off the automatic compressor management if requested by DS. Putting this
		// here is a convenience since this function is called at each mode change.
		// if (SmartDashboard.getBoolean("CompressorEnabled", true)) 
		// 	pcm.enableCompressorDigital();
		// else
		// 	pcm.disableCompressor();
		
		pdp.clearStickyFaults();
		//pcm.clearAllStickyFaults(); // Add back if we use a commpressor.
		
		if (monitorPDPThread != null) monitorPDPThread.reset();
    }

	public void fixPathPlannerGyro() {
		driveBase.fixPathPlannerGyro();
	}

	public void lockMechanisms() {
		elevShooter.elevator.lockPosition();
		elevShooter.shooter.lockPosition();
	}
	
	public void unlockMechanisms() {
		elevShooter.elevator.unlockPosition();
		elevShooter.shooter.unlockPosition();
		driverController.setRumble(RumbleType.kBothRumble, 0);
	}
	
	/**
     * Loads a PathPlanner path file into a path planner trajectory.
     * @param fileName Name of file. Will automatically look in deploy directory and add the .path ext.
     * @return The path's trajectory.
     */
    // public static PathPlannerTrajectory loadPPTrajectoryFile(String fileName)
    // {
    //     PathPlannerTrajectory  	trajectory;
    //     Path        			trajectoryFilePath;

	// 	// We fab up the full path for tracing but the loadPath() function does it's own
	// 	// thing constructing a path from just the filename.
	// 	trajectoryFilePath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/" + fileName + ".path");

	// 	Util.consoleLog("loading PP trajectory: %s", trajectoryFilePath);
		
	// 	trajectory = PathPlanner.loadPath(fileName,
	// 									  new PathConstraints(MAX_WHEEL_SPEED, MAX_WHEEL_ACCEL));

	// 	if (trajectory == null) 
	// 	{
	// 		Util.consoleLog("Unable to open pp trajectory: " + fileName);
	// 		throw new RuntimeException("Unable to open PP trajectory: " + fileName);
	// 	}

    //     Util.consoleLog("PP trajectory loaded: %s", fileName);

    //     return trajectory;
    // }

	// private void loadPPTestTrajectory()
	// {
	// 	ppTestTrajectory = loadPPTrajectoryFile("Test-Path");
	// }
}
