// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - matched to PathPlanner settings.json (maxDriveSpeed=3, defaultMaxAngVel=540 deg/s)
    public static final double kMaxSpeedMetersPerSecond = 0.3;       // m/s
    public static final double kMaxAngularSpeed = 0.25 * Math.PI;    // rad/s (~540 deg/s)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kFrontRightTurningCanId = 1;
    public static final int kRearRightTurningCanId = 7;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    /** Button for "go to AprilTag 9" (teleop): positions robot centered in front of tag 9. */
    public static final int kGoToAprilTag9Button = 1; // Xbox A
    /** Button for "go to AprilTag 10" (teleop): positions robot centered in front of tag 10. */
    public static final int kGoToAprilTag10Button = 2; // Xbox B
  }

  /** Limelight / AprilTag: targeting and go-to-tag. */
  public static final class VisionConstants {
    /** Max time (seconds) for GoToAprilTagCommand; then command ends (safety). */
    public static final double kGoToAprilTagTimeoutSeconds = 5.0;
    /** If tag not seen within this time (s), command ends (no tag). */
    public static final double kGoToAprilTagAcquireTimeoutSeconds = 1.0;
    /** Default distance (m) in front of AprilTag when using go-to-tag. */
    public static final double kDefaultDistanceFromTagMeters = 1.5;
    /**
     * AprilTag field poses from the official 2026-rebuilt-welded.json layout (WPIBlue frame).
     * Tags 9 and 10 are on the Red Reef structure, facing 0° (toward +X / Red wall).
     * Source: wpilibsuite/allwpilib 2026-rebuilt-welded.json
     */
    public static final Pose2d kTag1FieldPose  = new Pose2d(11.8779798, 7.4247756, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag2FieldPose  = new Pose2d(11.9154194, 4.638039999999999, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag3FieldPose  = new Pose2d(11.3118646, 4.3902376, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag4FieldPose  = new Pose2d(11.3118646, 4.0346376, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag5FieldPose  = new Pose2d(11.9154194, 3.4312351999999997, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag6FieldPose  = new Pose2d(11.8779798, 0.6444996, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag7FieldPose  = new Pose2d(11.9528844, 0.6444996, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag8FieldPose  = new Pose2d(12.2710194, 3.4312351999999997, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag9FieldPose  = new Pose2d(12.519177399999998, 3.6790375999999996, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag10FieldPose = new Pose2d(12.519177399999998, 4.0346376, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag11FieldPose = new Pose2d(12.2710194, 4.638039999999999, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag12FieldPose = new Pose2d(11.9528844, 7.4247756, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag13FieldPose = new Pose2d(16.5333172, 7.4033126, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag14FieldPose = new Pose2d(16.5333172, 6.9715126, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag15FieldPose = new Pose2d(16.5329616, 4.3235626, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag16FieldPose = new Pose2d(16.5329616, 3.8917626, Rotation2d.fromDegrees(0));

    public static final Pose2d kTag17FieldPose = new Pose2d(4.6630844, 0.6444996, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag18FieldPose = new Pose2d(4.6256194, 3.4312351999999997, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag19FieldPose = new Pose2d(5.229174199999999, 3.6790375999999996, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag20FieldPose = new Pose2d(5.229174199999999, 4.0346376, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag21FieldPose = new Pose2d(4.6256194, 4.638039999999999, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag22FieldPose = new Pose2d(4.6630844, 4.5881798, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag23FieldPose = new Pose2d(4.5881798, 7.424775, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag24FieldPose = new Pose2d(4.2700194, 4.638039999999999, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag25FieldPose = new Pose2d(4.0218614, 4.3902376, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag26FieldPose = new Pose2d(4.0218614, 4.0346376, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag27FieldPose = new Pose2d(4.2700194, 3.4312351999999997, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag28FieldPose = new Pose2d(4.5881798, 0.6444996, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag29FieldPose = new Pose2d(0.0077469999999999995, 0.6659626, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag30FieldPose = new Pose2d(0.0077469999999999995, 1.0977626, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag31FieldPose = new Pose2d(0.0080772, 3.7457125999999996, Rotation2d.fromDegrees(0));
    public static final Pose2d kTag32FieldPose = new Pose2d(0.0080772, 4.1775126, Rotation2d.fromDegrees(0));
    /** Limelight NetworkTable name. */
    public static final String kLimelightTableName = "limelight";
  }

  public static final class AutoConstants {
    // Matched to PathPlanner settings.json (defaultMaxVel=3, defaultMaxAccel=3, defaultMaxAngVel=540, defaultMaxAngAccel=720)
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = 1 * Math.PI;     // rad/s (~540 deg/s)
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 1 * Math.PI; // rad/s^2 (~720 deg/s^2)

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
