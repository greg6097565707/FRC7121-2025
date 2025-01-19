// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.math.Conversions;
import frc.robot.Constants.ModuleConstants;



import frc.robot.Configs;

// public class MAXSwerveModule {
//   private final TalonFX m_drivingSpark;
//   private final SparkMax m_turningSpark;

//   private final RelativeEncoder m_drivingEncoder;
//   private final AbsoluteEncoder m_turningEncoder;

//   private final SparkClosedLoopController m_drivingClosedLoopController;
//   private final SparkClosedLoopController m_turningClosedLoopController;

//   private double m_chassisAngularOffset = 0;
//   private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

//   /**
//    * Constructs a MAXSwerveModule and configures the driving and turning motor,
//    * encoder, and PID controller. This configuration is specific to the REV
//    * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
//    * Encoder.
//    */
//   public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
//     // m_drivingSpark = new TalonFX(drivingCANId, MotorType.kBrushless);
//     m_drivingSpark = new TalonFX(drivingCANId);
//     m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

//     // m_drivingEncoder = m_drivingSpark;
//     m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

//     m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
//     m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

//     // Apply the respective configurations to the SPARKS. Reset parameters before
//     // applying the configuration to bring the SPARK to a known good state. Persist
//     // the settings to the SPARK to avoid losing them on a power cycle.
//     m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
//         PersistMode.kPersistParameters);
//     m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
//         PersistMode.kPersistParameters);

//     m_chassisAngularOffset = chassisAngularOffset;
//     m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
//     m_drivingEncoder.setPosition(0);
//   }

//   /**
//    * Returns the current state of the module.
//    *
//    * @return The current state of the module.
//    */
//   public SwerveModuleState getState() {
//     // Apply chassis angular offset to the encoder position to get the position
//     // relative to the chassis.
//     return new SwerveModuleState(m_drivingEncoder.getVelocity(),
//         new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
//   }

//   /**
//    * Returns the current position of the module.
//    *
//    * @return The current position of the module.
//    */
//   public SwerveModulePosition getPosition() {
//     // Apply chassis angular offset to the encoder position to get the position
//     // relative to the chassis.
//     return new SwerveModulePosition(
//         m_drivingEncoder.getPosition(),
//         new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
//   }

//   /**
//    * Sets the desired state for the module.
//    *
//    * @param desiredState Desired state with speed and angle.
//    */
//   public void setDesiredState(SwerveModuleState desiredState) {
//     // Apply chassis angular offset to the desired state.
//     SwerveModuleState correctedDesiredState = new SwerveModuleState();
//     correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
//     correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

//     // Optimize the reference state to avoid spinning further than 90 degrees.
//     correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

//     // Command driving and turning SPARKS towards their respective setpoints.
//     m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
//     m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

//     m_desiredState = desiredState;
//   }

//   /** Zeroes all the SwerveModule encoders. */
//   public void resetEncoders() {
//     m_drivingEncoder.setPosition(0);
//   }
// }

public class MAXSwerveModule {

  public final TalonFX m_drivingTalon;
  public final SparkMax m_turningSparkMax;

  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_turningPIDController;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ModuleConstants.kS, ModuleConstants.kV,
      ModuleConstants.kA);

  private double m_chassisAngularOffset = 0;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, boolean drivingReversed,
      boolean turningReversed) {
    m_drivingTalon = new TalonFX(drivingCANId);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    // Same for TalonFX
    // m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    /* Configure TalonFX's Sensor Source for Pirmary PID */
    m_drivingTalon.setInverted(drivingReversed);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    // m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setPositionConversionFactor()
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(turningReversed);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);
    m_turningPIDController.

    // Set the PID gains for the driving motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!(P,I,D,FF)

    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_drivingTalon.setNeutralMode(ModuleConstants.kDrivingMotorNeutralMode);
    // No current limit is set for driving motor
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_drivingTalon.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        Conversions.falconToMPS(m_drivingTalon.getVelocity().getValue(), ModuleConstants.kWheelCircumferenceMeters,
            ModuleConstants.kDrivingMotorReduction),
        new Rotation2d(m_turningEncoder.getPosition() + m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        getDriveMotorPosition(),
        new Rotation2d(m_turningEncoder.getPosition() + m_chassisAngularOffset));
  }

  public double getMotorTemp()
  {
    return m_drivingTalon.getDeviceTemp().getValueAsDouble();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */

  SwerveModuleState lastState;

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle       Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

    if (Math.abs(desiredState.speedMetersPerSecond) < SwerveConstants.DriveConstants.kMaxSpeedMetersPerSecond * 0.0012) {
      desiredState.angle = getState().angle;
    }

    desiredState = optimize(desiredState,
        Rotation2d.fromRadians(m_turningEncoder.getPosition() + m_chassisAngularOffset));
    // Command driving and turning SPARKS MAX towards their respective setpoints.

    // Apply chassis angular offset to the desired state.
    // Set speed of Falcon
    setSpeed(desiredState, isOpenLoop);

    m_turningPIDController.setReference(desiredState.angle.getRadians() - m_chassisAngularOffset,
        CANSparkMax.ControlType.kPosition);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingTalon.setPosition(0);
  }

  /**
   * Calculates distance by drive motors encoder.
   *
   * @return Motor position in meters.
   */
  public double getDriveMotorPosition() {
    return Conversions.falconToMeters(m_drivingTalon.getPosition().getValue(),
        ModuleConstants.kWheelCircumferenceMeters, ModuleConstants.kDrivingMotorReduction);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / ModuleConstants.kmaxSpeed;
      m_drivingTalon.set(percentOutput);
    } else {
      double velocity = Conversions.falconToRPM(Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
          ModuleConstants.kWheelCircumferenceMeters, ModuleConstants.kDrivingMotorReduction), 1) / 60.0;
      m_drivingTalon.setControl(new MotionMagicVelocityDutyCycle(velocity, 0, false,
          feedforward.calculate(desiredState.speedMetersPerSecond), 0, true, false, false));
    }
  }
}
