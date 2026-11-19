package frc.robot.subsystems;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ApproachTagConstants;

public class Telemetry extends SubsystemBase {
  private final DriveSubsystem m_drive;
  private final NetworkTable m_limelight =
      NetworkTableInstance.getDefault().getTable(ApproachTagConstants.kLimelightTableName);

  private final StructPublisher<Pose2d> m_pose = pose("Telemetry/Drive/Pose");
  private final StructPublisher<Rotation2d> m_heading = NetworkTableInstance.getDefault()
      .getStructTopic("Telemetry/Drive/Heading", Rotation2d.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> m_measuredStates =
      moduleStates("Telemetry/Drive/ModuleStates/Measured");
  private final StructArrayPublisher<SwerveModuleState> m_desiredStates =
      moduleStates("Telemetry/Drive/ModuleStates/Setpoints");
  private final StructPublisher<ChassisSpeeds> m_chassisSpeeds = NetworkTableInstance.getDefault()
      .getStructTopic("Telemetry/Drive/ChassisSpeeds", ChassisSpeeds.struct).publish();
  private final DoublePublisher m_turnRate = number("Telemetry/Drive/TurnRateDegPerSec");
  private final DoublePublisher m_batteryVolts = number("Telemetry/Robot/BatteryVolts");

  private final BooleanPublisher m_visionHasTarget = NetworkTableInstance.getDefault()
      .getBooleanTopic("Telemetry/Vision/HasTarget").publish();
  private final DoublePublisher m_visionTagId = number("Telemetry/Vision/TagId");
  private final StructPublisher<Pose2d> m_visionPose = pose("Telemetry/Vision/BotPose");

  private final StructPublisher<Pose2d> m_pathCurrentPose = pose("Telemetry/PathPlanner/CurrentPose");
  private final StructPublisher<Pose2d> m_pathTargetPose = pose("Telemetry/PathPlanner/TargetPose");
  private final StructArrayPublisher<Pose2d> m_activePath = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Telemetry/PathPlanner/ActivePath", Pose2d.struct).publish();

  public Telemetry(DriveSubsystem drive) {
    m_drive = drive;
    PathPlannerLogging.setLogCurrentPoseCallback(m_pathCurrentPose::set);
    PathPlannerLogging.setLogTargetPoseCallback(m_pathTargetPose::set);
    PathPlannerLogging.setLogActivePathCallback(poses -> m_activePath.set(poses.toArray(new Pose2d[0])));
  }

  @Override
  public void periodic() {
    m_pose.set(m_drive.getPose());
    m_heading.set(Rotation2d.fromDegrees(m_drive.getHeading()));
    m_measuredStates.set(m_drive.getModuleStates());
    m_desiredStates.set(m_drive.getDesiredModuleStates());
    m_chassisSpeeds.set(m_drive.getRobotRelativeSpeeds());
    m_turnRate.set(m_drive.getTurnRate());
    m_batteryVolts.set(RobotController.getBatteryVoltage());

    boolean hasTarget = m_limelight.getEntry("tv").getDouble(0) >= 0.5;
    m_visionHasTarget.set(hasTarget);
    m_visionTagId.set(m_limelight.getEntry("tid").getDouble(-1));
    double[] botpose = m_limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
    if (hasTarget && botpose.length >= 6) {
      m_visionPose.set(new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5])));
    }
  }

  private static StructPublisher<Pose2d> pose(String topic) {
    return NetworkTableInstance.getDefault().getStructTopic(topic, Pose2d.struct).publish();
  }

  private static StructArrayPublisher<SwerveModuleState> moduleStates(String topic) {
    return NetworkTableInstance.getDefault().getStructArrayTopic(topic, SwerveModuleState.struct).publish();
  }

  private static DoublePublisher number(String topic) {
    return NetworkTableInstance.getDefault().getDoubleTopic(topic).publish();
  }
}
