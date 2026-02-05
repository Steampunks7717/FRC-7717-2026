// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Publisher for Advantage Scope: swerve module states (topic /SwerveStates). */
  private final StructArrayPublisher<SwerveModuleState> m_swerveStatesPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct)
          .publish();

  /** Publisher for Advantage Scope: robot pose for 2D/3D field (topic Odometry/Robot). */
  private final StructPublisher<Pose2d> m_posePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("Odometry/Robot", Pose2d.struct)
          .publish();

  /** Sim-only: pose integrated from chassis speeds so the robot moves in sim. */
  private Pose2d m_simPose = new Pose2d();

  /** Last commanded chassis speeds (robot-relative) for simulation integration. */
  private ChassisSpeeds m_lastChassisSpeeds = new ChassisSpeeds();

  /** Last time in seconds for sim integration dt. */
  private double m_lastSimTime = Timer.getFPGATimestamp();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // PathPlanner AutoBuilder: load robot config from GUI and configure path following.
    // If no config file exists (e.g. first run), autonomous will use WPILib fallback in RobotContainer.
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotRelativeSpeeds,
          this::driveRobotRelative,
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          this);
    } catch (Exception e) {
      // No PathPlanner config or autos yet; getAutonomousCommand() will use WPILib default.
    }
  }

  /** True when we are not on the real robot (desktop/sim); use integrated pose for Advantage Scope. */
  private static boolean useIntegratedPose() {
    return !RobotBase.isReal();
  }

  @Override
  public void periodic() {
    if (useIntegratedPose()) {
      // Not on real robot: integrate chassis speeds so the robot moves on 2D/3D field when you drive
      double now = Timer.getFPGATimestamp();
      double dt = Math.min(now - m_lastSimTime, 0.05); // cap dt in case of pause
      m_lastSimTime = now;
      double vx = m_lastChassisSpeeds.vxMetersPerSecond;
      double vy = m_lastChassisSpeeds.vyMetersPerSecond;
      double omega = m_lastChassisSpeeds.omegaRadiansPerSecond;
      double theta = m_simPose.getRotation().getRadians();
      double newX = m_simPose.getX() + (vx * Math.cos(theta) - vy * Math.sin(theta)) * dt;
      double newY = m_simPose.getY() + (vx * Math.sin(theta) + vy * Math.cos(theta)) * dt;
      m_simPose = new Pose2d(newX, newY, new Rotation2d(theta + omega * dt));
    } else {
      // Real robot: update odometry from gyro and module positions
      m_odometry.update(
          Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
          });
    }
    // Publish module states for Advantage Scope (Swerve tab)
    m_swerveStatesPublisher.set(
        new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        });
    // Publish pose for Advantage Scope (2D Field / 3D Field tab)
    m_posePublisher.set(getPose());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    if (useIntegratedPose()) {
      return m_simPose;
    }
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current robot-relative chassis speeds (vx, vy, omega).
   * For path following and telemetry; velocities are in the robot frame.
   *
   * @return Current ChassisSpeeds in robot-relative frame (m/s, rad/s).
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Drives the robot using robot-relative ChassisSpeeds.
   * PathPlanner and path-following commands use this method; speeds are robot-relative.
   *
   * @param speeds Desired chassis speeds in robot frame (m/s, rad/s).
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    m_lastChassisSpeeds = speeds;
    var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(states);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    if (useIntegratedPose()) {
      m_simPose = pose;
    } else {
      m_odometry.resetPosition(
          Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
          },
          pose);
    }
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    // When not on real robot the gyro doesn't update; use current pose rotation for field-relative
    Rotation2d heading = useIntegratedPose()
        ? getPose().getRotation()
        : Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ));
    ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, heading)
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    m_lastChassisSpeeds = chassisSpeeds;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
