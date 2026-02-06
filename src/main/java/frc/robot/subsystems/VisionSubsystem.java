// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * Reads Limelight data via NetworkTables and provides target pose for AprilTag go-to.
 * The drive does not know about Limelight; commands use this subsystem to compute
 * setpoints and pass them to the drive.
 */
public class VisionSubsystem extends SubsystemBase {
  private final NetworkTable m_limelight = NetworkTableInstance.getDefault()
      .getTable(VisionConstants.kLimelightTableName);

  public VisionSubsystem() {}

  /** Horizontal offset from crosshair to target (degrees). */
  public double getTx() {
    return m_limelight.getEntry("tx").getDouble(0);
  }

  /** Vertical offset from crosshair to target (degrees). */
  public double getTy() {
    return m_limelight.getEntry("ty").getDouble(0);
  }

  /** Whether a target is present (0 or 1). */
  public boolean hasTarget() {
    return m_limelight.getEntry("tv").getDouble(0) >= 0.5;
  }

  /** AprilTag ID of the current target, or -1 if none. */
  public int getTargetId() {
    return (int) m_limelight.getEntry("tid").getDouble(-1);
  }

  /** Returns the field pose of the given AprilTag from the layout. */
  public Pose2d getTagPoseFromLayout(int tagId) {
    return switch (tagId) {
      case 9 -> VisionConstants.kTag9FieldPose;
      case 10 -> VisionConstants.kTag10FieldPose;
      default -> null;
    };
  }

  /**
   * Desired robot pose: centered in front of the tag at the given distance (meters).
   * "In front" means along the tag's facing direction (tag faces away from wall).
   */
  public Pose2d getTargetPoseInFrontOfTag(int tagId, double distanceMeters) {
    Pose2d tagPose = getTagPoseFromLayout(tagId);
    if (tagPose == null) return null;
    double angle = tagPose.getRotation().getRadians();
    // Robot sits "in front" of tag = tag position minus distance along tag's forward direction
    double x = tagPose.getX() - distanceMeters * Math.cos(angle);
    double y = tagPose.getY() - distanceMeters * Math.sin(angle);
    return new Pose2d(x, y, tagPose.getRotation());
  }

  /** True if the current target is the given tag ID. */
  public boolean isSeeingTag(int tagId) {
    return hasTarget() && getTargetId() == tagId;
  }
}
