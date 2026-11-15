package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ApproachTagConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ApproachTagCommand extends Command {
  private final DriveSubsystem m_drive;
  private final NetworkTable m_limelight =
      NetworkTableInstance.getDefault().getTable(ApproachTagConstants.kLimelightTableName);
  private boolean m_atGoal;
  private double m_lastHeartbeat;
  private int m_staleCycles;

  public ApproachTagCommand(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_atGoal = false;
    m_lastHeartbeat = m_limelight.getEntry("hb").getDouble(-1);
    m_staleCycles = ApproachTagConstants.kMaxStaleCycles;
  }

  @Override
  public void execute() {
    double heartbeat = m_limelight.getEntry("hb").getDouble(-1);
    if (heartbeat != m_lastHeartbeat) {
      m_lastHeartbeat = heartbeat;
      m_staleCycles = 0;
    } else {
      m_staleCycles++;
    }
    if (m_staleCycles >= ApproachTagConstants.kMaxStaleCycles) {
      m_drive.drive(0, 0, 0, false);
      SmartDashboard.putString("ApproachTag/Estado", "camara sin datos");
      return;
    }

    boolean hasTarget = m_limelight.getEntry("tv").getDouble(0) >= 0.5;
    double[] targetInCamera =
        m_limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);
    if (!hasTarget || targetInCamera.length < 3
        || !Double.isFinite(targetInCamera[2])
        || targetInCamera[2] <= ApproachTagConstants.kMinValidDistanceMeters) {
      m_drive.drive(0, 0, 0, false);
      SmartDashboard.putString("ApproachTag/Estado", "sin tag");
      return;
    }

    double distanceMeters = targetInCamera[2];
    double txDegrees = m_limelight.getEntry("tx").getDouble(0);
    ApproachTagController.Output output = ApproachTagController.calculate(distanceMeters, txDegrees);
    m_drive.drive(output.forward(), 0, output.rotation(), false);

    SmartDashboard.putNumber("ApproachTag/TagId", m_limelight.getEntry("tid").getDouble(-1));
    SmartDashboard.putNumber("ApproachTag/DistanciaM", distanceMeters);
    SmartDashboard.putNumber("ApproachTag/TxGrados", txDegrees);
    SmartDashboard.putString("ApproachTag/Estado", output.atGoal() ? "a 1 m" : "acercandose");
    m_atGoal = output.atGoal();
  }

  @Override
  public boolean isFinished() {
    return m_atGoal;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    if (interrupted) {
      SmartDashboard.putString("ApproachTag/Estado", "cancelado");
    }
  }
}
