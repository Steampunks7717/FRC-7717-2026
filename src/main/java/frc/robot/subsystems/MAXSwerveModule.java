// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Configs;

public class MAXSwerveModule {
  // Null in simulation — hardware not available on desktop
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;
  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;
  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule. On a real robot, initializes SparkMax hardware.
   * In simulation, all motor objects are null and methods return safe defaults.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_chassisAngularOffset = chassisAngularOffset;

    if (RobotBase.isReal()) {
      m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
      m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
      m_drivingEncoder = m_drivingSpark.getEncoder();
      m_turningEncoder = m_turningSpark.getAbsoluteEncoder();
      m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
      m_turningClosedLoopController = m_turningSpark.getClosedLoopController();
      m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig,
          ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig,
          ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
      m_drivingEncoder.setPosition(0);
    } else {
      m_drivingSpark = null;
      m_turningSpark = null;
      m_drivingEncoder = null;
      m_turningEncoder = null;
      m_drivingClosedLoopController = null;
      m_turningClosedLoopController = null;
    }
  }

  /**
   * Returns the current state of the module.
   * In simulation, returns the last commanded state so visualizations work.
   */
  public SwerveModuleState getState() {
    if (!RobotBase.isReal()) {
      return new SwerveModuleState(
          m_desiredState.speedMetersPerSecond,
          m_desiredState.angle);
    }
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   * In simulation, returns angle from last command and zero distance.
   */
  public SwerveModulePosition getPosition() {
    if (!RobotBase.isReal()) {
      return new SwerveModulePosition(0.0, m_desiredState.angle);
    }
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   * In simulation, stores the state for getState() but skips motor commands.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    m_desiredState = desiredState;

    if (!RobotBase.isReal()) return;

    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));
    m_drivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    if (!RobotBase.isReal()) return;
    m_drivingEncoder.setPosition(0);
  }
}
