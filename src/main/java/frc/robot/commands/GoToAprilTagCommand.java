// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Teleop: navigates the robot to a pose centered in front of a given AprilTag.
 *
 * Uses PathPlanner's pathfindToPose() so the swerve properly decouples
 * translation from heading and avoids obstacles using the navgrid.
 *
 * If the tag is not in the known layout (null pose), the command ends immediately.
 */
public class GoToAprilTagCommand extends Command {
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private final int m_tagId;
  private final double m_distanceMeters;

  /** Inner PathPlanner pathfind command, built on initialize(). Null if tag unknown. */
  private Command m_pathfindCommand = null;

  public GoToAprilTagCommand(DriveSubsystem drive, VisionSubsystem vision, int tagId) {
    this(drive, vision, tagId, VisionConstants.kDefaultDistanceFromTagMeters);
  }

  public GoToAprilTagCommand(DriveSubsystem drive, VisionSubsystem vision, int tagId,
      double distanceMeters) {
    m_drive = drive;
    m_vision = vision;
    m_tagId = tagId;
    m_distanceMeters = distanceMeters;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_pathfindCommand = null;

    Pose2d target = m_vision.getTargetPoseInFrontOfTag(m_tagId, m_distanceMeters);
    if (target == null) return; // tag not in layout → end immediately

    PathConstraints constraints = new PathConstraints(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        AutoConstants.kMaxAngularSpeedRadiansPerSecond,
        AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

    m_pathfindCommand = AutoBuilder.pathfindToPose(target, constraints);
    m_pathfindCommand.initialize();
  }

  @Override
  public void execute() {
    if (m_pathfindCommand != null) {
      m_pathfindCommand.execute();
    }
  }

  @Override
  public boolean isFinished() {
    if (m_pathfindCommand == null) return true;
    return m_pathfindCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (m_pathfindCommand != null) {
      m_pathfindCommand.end(interrupted);
    }
    m_drive.driveRobotRelative(new ChassisSpeeds());
  }
}
