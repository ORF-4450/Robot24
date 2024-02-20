
package Team4450.Robot24;

import java.util.Properties;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
	public static String		PROGRAM_NAME = "ORF24-02.20.24-1";

	public static Robot			robot;

	public static Properties	robotProperties;
	  
	public static boolean		isClone = false, isComp = false, tracing = false;
	    	
	public static DriverStation.Alliance	 alliance;
	public static int                        location, matchNumber;
	public static String					 eventName, gameMessage;
	    
	// Non-drive base motor controller port assignments

    // INTAKE ======
    public static final int     INTAKE_MOTOR_1 = 9;
    public static final int     INTAKE_MOTOR_2 = 10;
    public static final double  INTAKE_SPEED = 1;

    // SHOOTER ======
    public static final int     SHOOTER_MOTOR_TOP = 11;
    public static final int     SHOOTER_MOTOR_BOTTOM = 12;
    public static final int     SHOOTER_MOTOR_FEEDER = 13;
    public static final int     SHOOTER_MOTOR_PIVOT = 14;
    // public static final int     SHOOTER_MOTOR_PIVOT_2 = 15; // if we have to add one
    public static final double  SHOOTER_SPEED = 1;
    public static final double  SHOOTER_FEED_SPEED = 1;

    // multiplied by motor rotations to get degrees of shooter angle
    public static final double  SHOOTER_PIVOT_FACTOR = (1.0 / (765.0 / 13.0)) * 360;
    
    // ELEVATOR
    public static final int     ELEVATOR_MOTOR_RIGHT = 16;
    public static final int     ELEVATOR_MOTOR_LEFT = 17;
    public static final int     ELEVATOR_MOTOR_INNER = 18;

    // CAMERAS
    public static Transform3d   CAMERA_POSE_TRANSFORM = new Transform3d(
        new Translation3d(0, 0, 0),
        new Rotation3d(0, 0, 0)
    );
    
    public static Transform3d   CAMERA_BACK_TRANSFORM = new Transform3d(
        new Translation3d(0, 0, 1),
        new Rotation3d(0, Math.toRadians(-15), 0)
    );
    
    public static Transform3d   CAMERA_FRONT_TRANSFORM = new Transform3d(
        new Translation3d(0, 0, 1.5),
        new Rotation3d(0, Math.toRadians(45), 0)
    );

    // the names of the cameras in the PhotonVision software
    public static String        CAMERA_POSE_ESTIMATOR = "";
    public static String        CAMERA_FRONT = "4450-LL";
    public static String        CAMERA_BACK = "back";

    public static final int     REV_PDB = 20;
	
	// GamePad port assignments.
	public static final int		DRIVER_PAD = 0, UTILITY_PAD = 1;
    public static final double  DRIVE_DEADBAND = 0.05, ROTATION_DEADBAND = .05;

	// Pneumatic valve controller port assignments.
	//public static final int		COMPRESSOR = 0;

	// Digital Input port assignments. Encoder takes 2 ports.
    public static final int     NOTE_SENSOR_INTAKE = 0;
    public static final int     NOTE_SENSOR_SHOOTER = 1;
	  
	// Analog Input port assignments.
	
	// LCD display line number constants showing class where the line is set.
	public static final int		LCD_1 = 1;	    // Robot, Auto Commands.
	public static final int		LCD_2 = 2;	    // Swerve Drive command.
	public static final int		LCD_3 = 3;	    // ShuffleBoard subsystem.
	public static final int		LCD_4 = 4;	    // ShuffleBoard subsystem.
	public static final int		LCD_5 = 5;	    // Autonomous commands.
	public static final int		LCD_6 = 6;	    // ShuffleBoard subsystem.
	public static final int		LCD_7 = 7;	    // ShuffleBoard subsystem.
	public static final int		LCD_8 = 8;	    // ShuffleBoard subsystem.
	public static final int		LCD_9 = 9;	    // ShuffleBoard subsystem.
	public static final int		LCD_10 = 10;	// ShuffleBoard subsystem.

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds.
        public static final double kMaxSpeedMetersPerSecond = 2.0;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second.
        public static final double kSlowModeFactor = .15; // 15% of normal.

        public static final double kDirectionSlewRate = 1.2; // radians per second.
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%).
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%).

        // Chassis configuration

        // Distance between centers of right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(29);

        // Distance between front and back wheel centers
        public static final double kWheelBase = Units.inchesToMeters(29);

        // Drive base radius in meters. Distance from robot center to furthest module.
        public static final double kDriveBaseRadius = .54;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians at
        // alignment/start up.
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 1;
        public static final int kFrontLeftTurningCanId = 2;

        public static final int kFrontRightDrivingCanId = 3;
        public static final int kFrontRightTurningCanId = 4;

        public static final int kRearLeftDrivingCanId = 5;
        public static final int kRearLeftTurningCanId = 6;

        public static final int kRearRightDrivingCanId = 7;
        public static final int kRearRightTurningCanId = 8;

        public static final boolean kGyroReversed = false;

        // Default starting field position in meters for pose tracking. 2024 field. 
        // TODO: This likely will need futher definition as to how we will manage starting points.
        public static final Pose2d	DEFAULT_STARTING_POSE = new Pose2d(.697, 7.153, Rotation2d.fromDegrees(0));
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction; // meters

        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 1.0; // High to mitigate rotational drift.
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        public static final double kHolonomicPathFollowerP = 5.0;
        
        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

  //-------------------- No student code above this line ------------------------------------------------------

}
