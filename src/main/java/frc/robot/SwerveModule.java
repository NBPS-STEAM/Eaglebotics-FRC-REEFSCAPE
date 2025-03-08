// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import swervelib.encoders.CanAndMagSwerve;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SwerveModule {
  private final TalonFX m_drivingMotor;
  private final SparkMax m_turningMotor;

  private final EncoderCompat m_turningEncoder;

  private final PIDController m_turningPID;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(int drivingCANId, int turningCANId, int encoderCANId, double chassisAngularOffset, String encoderType) {
    m_drivingMotor = new TalonFX(drivingCANId, "swerve");
    m_turningMotor = new SparkMax(turningCANId, MotorType.kBrushless);

    switch (encoderType) {
      case "canandmag_can":
        m_turningEncoder = new CanandmagCompat(encoderCANId);
        break;
      case "cancoder":
        m_turningEncoder = new CancoderCompat(encoderCANId, "rio");
        break;
      default:
        throw new RuntimeException("Invalid Swerve Encoder Type");
    }

    m_turningPID = new PIDController(DriveConstants.kTurningP, DriveConstants.kTurningI, DriveConstants.kTurningD);
    m_turningPID.enableContinuousInput(0, 2 * Math.PI);

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    //m_drivingMotor.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
    //    PersistMode.kPersistParameters);
    m_drivingMotor.setNeutralMode(NeutralModeValue.Brake);
    m_turningMotor.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPositionCpt());
    m_drivingMotor.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingMotor.getVelocity().getValueAsDouble(),
        new Rotation2d(m_turningEncoder.getPositionCpt() - m_chassisAngularOffset));
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
      m_drivingMotor.getPosition().getValueAsDouble(),
        new Rotation2d(m_turningEncoder.getPositionCpt() - m_chassisAngularOffset));
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
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPositionCpt()));

    // Command driving and turning SPARKS towards their respective setpoints.
    //m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_drivingMotor.setControl(new VelocityVoltage(correctedDesiredState.speedMetersPerSecond)); // might work?
    //m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
    m_turningPID.setSetpoint(correctedDesiredState.angle.getRadians());
    m_turningMotor.set(m_turningPID.calculate(m_turningEncoder.getPositionCpt()));

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingMotor.setPosition(0);
  }



  interface EncoderCompat {
    /** Encoder position within [0, 2*PI) */
    double getPositionCpt();
  }

  class CanandmagCompat extends CanAndMagSwerve implements EncoderCompat {
    public CanandmagCompat(int canid) {
      super(canid);
    }

    @Override
    public double getPositionCpt() {
      return (getAbsolutePosition() / 180) * Math.PI;
    }
  }

  class CancoderCompat extends CANcoder implements EncoderCompat {
    public CancoderCompat(int deviceId, String canbus) {
      super(deviceId, canbus);
    }

    @Override
    public double getPositionCpt() {
      double ang = getAbsolutePosition().getValueAsDouble();
      if (ang < 0) ang += 1;
      return ang * 2 * Math.PI;
    }
  }
}
