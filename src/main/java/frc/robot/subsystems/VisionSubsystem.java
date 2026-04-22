// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * Reads Limelight data via NetworkTables (MegaTag2 / botpose_orb_wpiblue).
 *
 * Each periodic cycle:
 *  1. Sends the current robot yaw to the Limelight so MegaTag2 can fuse it.
 *  2. Reads botpose_orb_wpiblue (robot pose with latency-compensated timestamp).
 *  3. Passes the measurement to DriveSubsystem.addVisionMeasurement() for fusion.
 *
 * Commands use this subsystem to compute the target pose in front of an AprilTag.
 */
public class VisionSubsystem extends SubsystemBase {
  private final NetworkTable m_limelight = NetworkTableInstance.getDefault()
      .getTable(VisionConstants.kLimelightTableName);

  private final DriveSubsystem m_drive;

  public VisionSubsystem(DriveSubsystem drive) {
    m_drive = drive;
  }

  @Override
  public void periodic() {
    // Step 1: send robot yaw to Limelight so MegaTag2 can anchor orientation.
    // This must be done BEFORE reading botpose_orb_wpiblue every cycle.
    m_limelight.getEntry("robot_orientation_set")
        .setDoubleArray(new double[] { m_drive.getHeading(), 0, 0, 0, 0, 0 });

    // Step 2: read MegaTag2 botpose. Array layout:
    //   [0]=x(m), [1]=y(m), [2]=z(m), [3]=roll, [4]=pitch, [5]=yaw(deg), [6]=latency(ms)
    double[] botpose = m_limelight.getEntry("botpose_orb_wpiblue")
        .getDoubleArray(new double[0]);

    boolean targetSeen = hasTarget();
    int targetId = getTargetId();

    // ── SmartDashboard logging ────────────────────────────────────────────────
    double rawTv = m_limelight.getEntry("tv").getDouble(-999);
    double rawTid = m_limelight.getEntry("tid").getDouble(-999);
    SmartDashboard.putNumber("rawTv", rawTv);
    SmartDashboard.putNumber("rawTid", rawTid);
    SmartDashboard.putString("TableName", VisionConstants.kLimelightTableName);
    SmartDashboard.putBoolean("Vision/Nt_Connected", NetworkTableInstance.getDefault().isConnected());
    SmartDashboard.putBoolean("Vision/HasTarget", targetSeen);
    SmartDashboard.putNumber("Vision/TargetId", targetId);
    SmartDashboard.putNumber("Vision/tx", getTx());
    SmartDashboard.putNumber("Vision/ty", getTy());

    // Step 3: fuse into pose estimator only when measurement is valid and fresh
    if (targetSeen && botpose.length >= 7) {
      double latencyMs  = botpose[6];
      Pose2d visionPose = new Pose2d(botpose[0], botpose[1],
          Rotation2d.fromDegrees(botpose[5]));
      double timestamp  = Timer.getFPGATimestamp() - (latencyMs / 1000.0);

      SmartDashboard.putNumber("Vision/BotPose_X", visionPose.getX());
      SmartDashboard.putNumber("Vision/BotPose_Y", visionPose.getY());
      SmartDashboard.putNumber("Vision/BotPose_Yaw", visionPose.getRotation().getDegrees());
      SmartDashboard.putNumber("Vision/LatencyMs", latencyMs);

      // Reject stale or zero poses (limelight not ready / no lock)
      boolean poseValid = latencyMs < 100
          && (visionPose.getX() != 0 || visionPose.getY() != 0);
      SmartDashboard.putBoolean("Vision/PoseValid", poseValid);

      if (poseValid) {
        m_drive.addVisionMeasurement(visionPose, timestamp);
      }
    }
  }

  // ── Raw Limelight getters ──────────────────────────────────────────────────

  /** Horizontal offset from crosshair to target (degrees). */
  public double getTx() {
    return m_limelight.getEntry("tx").getDouble(0);
  }

  /** Vertical offset from crosshair to target (degrees). */
  public double getTy() {
    return m_limelight.getEntry("ty").getDouble(0);
  }

  /** Whether a target is present (tv == 1). */
  public boolean hasTarget() {
    return m_limelight.getEntry("tv").getDouble(0) >= 0.5;
  }

  /** AprilTag ID of the current primary target, or -1 if none. */
  public int getTargetId() {
    return (int) m_limelight.getEntry("tid").getDouble(-1);
  }

  /** True if the current primary target is the given tag ID. */
  public boolean isSeeingTag(int tagId) {
    return hasTarget() && getTargetId() == tagId;
  }

  // ── Pose helpers for navigation ───────────────────────────────────────────

  /** Returns the field pose of the given AprilTag from the 2026 Rebuilt layout. */
  public Pose2d getTagPoseFromLayout(int tagId) {
    return switch (tagId) {
      case 1  -> VisionConstants.kTag1FieldPose;
      case 2  -> VisionConstants.kTag2FieldPose;
      case 3  -> VisionConstants.kTag3FieldPose;
      case 4  -> VisionConstants.kTag4FieldPose;
      case 5  -> VisionConstants.kTag5FieldPose;
      case 6  -> VisionConstants.kTag6FieldPose;
      case 7  -> VisionConstants.kTag7FieldPose;
      case 8  -> VisionConstants.kTag8FieldPose;
      case 9  -> VisionConstants.kTag9FieldPose;
      case 10 -> VisionConstants.kTag10FieldPose;
      case 11 -> VisionConstants.kTag11FieldPose;
      case 12 -> VisionConstants.kTag12FieldPose;
      case 13 -> VisionConstants.kTag13FieldPose;
      case 14 -> VisionConstants.kTag14FieldPose;
      case 15 -> VisionConstants.kTag15FieldPose;
      case 16 -> VisionConstants.kTag16FieldPose;
      case 17 -> VisionConstants.kTag17FieldPose;
      case 18 -> VisionConstants.kTag18FieldPose;
      case 19 -> VisionConstants.kTag19FieldPose;
      case 20 -> VisionConstants.kTag20FieldPose;
      case 21 -> VisionConstants.kTag21FieldPose;
      case 22 -> VisionConstants.kTag22FieldPose;
      case 23 -> VisionConstants.kTag23FieldPose;
      case 24 -> VisionConstants.kTag24FieldPose;
      case 25 -> VisionConstants.kTag25FieldPose;
      case 26 -> VisionConstants.kTag26FieldPose;
      case 27 -> VisionConstants.kTag27FieldPose;
      case 28 -> VisionConstants.kTag28FieldPose;
      case 29 -> VisionConstants.kTag29FieldPose;
      case 30 -> VisionConstants.kTag30FieldPose;
      case 31 -> VisionConstants.kTag31FieldPose;
      case 32 -> VisionConstants.kTag32FieldPose;
      default -> null;
    };
  }

  /**
   * Desired robot pose: centered in front of the tag at the given distance (meters),
   * facing toward the tag (180° from the tag's own facing direction).
   *
   * @param tagId         Target AprilTag ID.
   * @param distanceMeters Distance to stand in front of the tag.
   * @return Target Pose2d, or null if the tag ID is not in the layout.
   */
  public Pose2d getTargetPoseInFrontOfTag(int tagId, double distanceMeters) {
    Pose2d tagPose = getTagPoseFromLayout(tagId);
    if (tagPose == null) return null;

    double angle = tagPose.getRotation().getRadians();
    // Position: step back from the tag along the tag's facing direction
    double x = tagPose.getX() - distanceMeters * Math.cos(angle);
    double y = tagPose.getY() - distanceMeters * Math.sin(angle);
    // Robot must face the tag → rotate tag's heading by 180°
    Rotation2d robotHeading = tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
    return new Pose2d(x, y, robotHeading);
  }
}
