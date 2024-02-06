// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package Team4450.Robot24.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import Team4450.Lib.Util;

import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;

import Team4450.Robot24.Constants.ModuleConstants;

public class MAXSwerveModule implements Sendable {
  private final CANSparkMax drivingSparkMax;
  private final CANSparkMax turningSparkMax;

  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkPIDController drivingPIDController;
  private final SparkPIDController turningPIDController;

  private double            chassisAngularOffset = 0;

  private double            currentSimVelocity = 0, currentSimPosition = 0, currentSimAngle = 0;
  public String             moduleLocation;
  private Pose2d            pose;
  private Translation2d     translation2d;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, String moduleLocation) {
    this.moduleLocation = moduleLocation;

    Util.consoleLog("%s", moduleLocation);
               
    SendableRegistry.addLW(this, "DriveBase/Swerve Modules", moduleLocation);
    
    drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    drivingSparkMax.restoreFactoryDefaults();
    turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    drivingEncoder = drivingSparkMax.getEncoder();
    turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    drivingPIDController = drivingSparkMax.getPIDController();
    turningPIDController = turningSparkMax.getPIDController();
    drivingPIDController.setFeedbackDevice(drivingEncoder);
    turningPIDController.setFeedbackDevice(turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    drivingPIDController.setP(ModuleConstants.kDrivingP);
    drivingPIDController.setI(ModuleConstants.kDrivingI);
    drivingPIDController.setD(ModuleConstants.kDrivingD);
    drivingPIDController.setFF(ModuleConstants.kDrivingFF);

    drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
                                        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    turningPIDController.setP(ModuleConstants.kTurningP);
    turningPIDController.setI(ModuleConstants.kTurningI);
    turningPIDController.setD(ModuleConstants.kTurningD);
    turningPIDController.setFF(ModuleConstants.kTurningFF);
    
    turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
                                        ModuleConstants.kTurningMaxOutput);

    drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    drivingSparkMax.burnFlash();
    turningSparkMax.burnFlash();

    this.chassisAngularOffset = chassisAngularOffset;
    
    // if (RobotBase.isSimulation())
    //   desiredState.angle = new Rotation2d();
    // else
    //   desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    
    drivingEncoder.setPosition(0);
    
    if (RobotBase.isSimulation()) 
    {
      // Note that the REV simulation does not work correctly. We have hacked
      // a solution where we drive the sim through our code, not by reading the
      // REV simulated encoder position and velocity, which are incorrect. However, 
      // registering the motor controller with the REV sim is still needed.

      REVPhysicsSim.getInstance().addSparkMax(turningSparkMax, DCMotor.getNeo550(1));
      REVPhysicsSim.getInstance().addSparkMax(drivingSparkMax, DCMotor.getNEO(1));
    }
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.

    if (RobotBase.isReal())
      return new SwerveModuleState(drivingEncoder.getVelocity(),
          new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    else
      return new SwerveModuleState(currentSimVelocity,
          new Rotation2d(currentSimAngle - chassisAngularOffset));
  } 

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    SwerveModulePosition  position;

    if (RobotBase.isReal())
      position = new SwerveModulePosition(
          drivingEncoder.getPosition(),
          new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    else
      position = new SwerveModulePosition(
          currentSimPosition,
          new Rotation2d(currentSimAngle - chassisAngularOffset));
    
    return position;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    //SmartDashboard.putNumber(moduleLocation + " desired speed", correctedDesiredState.speedMetersPerSecond);

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    currentSimAngle = optimizedDesiredState.angle.getRadians();

    currentSimVelocity = optimizedDesiredState.speedMetersPerSecond;
    
    double distancePer20Ms = currentSimVelocity / 50.0;

    currentSimPosition += distancePer20Ms;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }

  /**
   * Sets module pose (not robot pose). Used for field2d display.
   * @param pose The module pose to set.
   */
  public void setModulePose(Pose2d pose) 
  {
      this.pose = pose;
  }

  /**
   * Returns the module pose (Not robot pose).
   * @return Module pose.
   */
  public Pose2d getPose()
  {
      //SmartDashboard.putString(moduleLocation + " pose", pose.toString());

      return pose;
  }
  
  /**
   * Returns the module steering angle.
   * @return Steering angle
   */
  public Rotation2d getAngle2d() 
  {
      Rotation2d  rot;

      if (RobotBase.isReal())
          rot = new Rotation2d(turningEncoder.getPosition());
      else
          rot = new Rotation2d(currentSimAngle - chassisAngularOffset);

      return rot;
  }

  public void setTranslation2d(Translation2d translation) 
  {
      translation2d = translation;            
  }

  public Translation2d getTranslation2d() 
  {
      return translation2d;
  }

  public void setBrakeMode(boolean on)
  {
    if (on)
      drivingSparkMax.setIdleMode(IdleMode.kBrake);
    else
      drivingSparkMax.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Return current drive velocity
   * @return Velocity m/s.
   */
  public double getVelocity()
  {
    if (RobotBase.isReal())
      return drivingEncoder.getVelocity();
    else
      return currentSimVelocity;
  }

  @Override
	public void initSendable( SendableBuilder builder )
	{
    builder.setSmartDashboardType(getClass().getSimpleName());

    builder.addDoubleProperty("1 Cur pos dist", () -> getPosition().distanceMeters, null);
    builder.addDoubleProperty("2 Cur pos angle", () -> getPosition().angle.getDegrees(), null);
    builder.addStringProperty("3 Pose", () -> getPose().toString(), null);
    builder.addDoubleProperty("4 Velocity SP", () -> currentSimVelocity, null);
    builder.addDoubleProperty("5 Steer angle SP", () -> Math.toDegrees(currentSimAngle), null);
    builder.addDoubleProperty("6 Actual velocity", () -> getVelocity(), null);
    builder.addDoubleProperty("7 Actual steer sngle", () -> getAngle2d().getDegrees(), null);
	}   
}
