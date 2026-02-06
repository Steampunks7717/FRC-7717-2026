// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Teleop: positions the robot centered in front of an AprilTag (9 or 10) at a set distance.
 * - Timeout: ends after {@link VisionConstants#kGoToAprilTagTimeoutSeconds}.
 * - No tag / wrong tag: ends immediately without blocking.
 * - Uses a trajectory for fast, efficient motion; drive only receives setpoints.
 */
public class GoToAprilTagCommand extends Command {
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private final int m_tagId;
  private final double m_distanceMeters;
  private final double m_timeoutSeconds;

  private final Timer m_timer = new Timer();
  private final PIDController m_xController = new PIDController(AutoConstants.kPXController, 0, 0);
  private final PIDController m_yController = new PIDController(AutoConstants.kPYController, 0, 0);
  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  private Trajectory m_trajectory;
  private double m_trajectoryStartTime;
  private boolean m_hasStartedTrajectory;

  public GoToAprilTagCommand(DriveSubsystem drive, VisionSubsystem vision, int tagId) {
    this(drive, vision, tagId, VisionConstants.kDefaultDistanceFromTagMeters);
  }

  public GoToAprilTagCommand(DriveSubsystem drive, VisionSubsystem vision, int tagId, double distanceMeters) {
    m_drive = drive;
    m_vision = vision;
    m_tagId = tagId;
    m_distanceMeters = distanceMeters;
    m_timeoutSeconds = VisionConstants.kGoToAprilTagTimeoutSeconds;
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive, vision);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_hasStartedTrajectory = false;
    m_trajectory = null;
  }

  @Override
  public void execute() {
    if (m_timer.get() > m_timeoutSeconds) {
      return;
    }

    if (!m_vision.isSeeingTag(m_tagId)) {
      if (!m_hasStartedTrajectory) {
        m_drive.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        return;
      }
      // Already following trajectory; keep going (no need to keep seeing tag)
    } else {
      if (!m_hasStartedTrajectory) {
        Pose2d target = m_vision.getTargetPoseInFrontOfTag(m_tagId, m_distanceMeters);
        if (target == null) return;
        Pose2d current = m_drive.getPose();
        TrajectoryConfig config = new TrajectoryConfig(
            Math.min(DriveConstants.kMaxSpeedMetersPerSecond, 3.5),
            Math.min(4.0, 3.5))
            .setKinematics(DriveConstants.kDriveKinematics);
        m_trajectory = TrajectoryGenerator.generateTrajectory(
            current,
            java.util.List.of(),
            target,
            config);
        m_trajectoryStartTime = Timer.getFPGATimestamp();
        m_hasStartedTrajectory = true;
      }
    }

    if (!m_hasStartedTrajectory || m_trajectory == null) return;

    double t = Timer.getFPGATimestamp() - m_trajectoryStartTime;
    if (t >= m_trajectory.getTotalTimeSeconds()) {
      m_drive.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
      return;
    }

    State s = m_trajectory.sample(t);
    double vx = s.velocityMetersPerSecond * s.poseMeters.getRotation().getCos();
    double vy = s.velocityMetersPerSecond * s.poseMeters.getRotation().getSin();
    Pose2d current = m_drive.getPose();
    double omega = m_thetaController.calculate(
        current.getRotation().getRadians(),
        s.poseMeters.getRotation().getRadians());

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        vx + m_xController.calculate(current.getX(), s.poseMeters.getX()),
        vy + m_yController.calculate(current.getY(), s.poseMeters.getY()),
        omega,
        current.getRotation());
    m_drive.driveRobotRelative(speeds);
  }

  @Override
  public boolean isFinished() {
    if (m_timer.get() > m_timeoutSeconds) return true;
    if (!m_vision.isSeeingTag(m_tagId) && !m_hasStartedTrajectory
        && m_timer.get() > VisionConstants.kGoToAprilTagAcquireTimeoutSeconds) return true;
    if (m_hasStartedTrajectory && m_trajectory != null) {
      double t = Timer.getFPGATimestamp() - m_trajectoryStartTime;
      if (t >= m_trajectory.getTotalTimeSeconds()) return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
  }
}
