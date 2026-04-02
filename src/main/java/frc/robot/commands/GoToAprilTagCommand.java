// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TagGroups;
import frc.robot.TagGroups.TagGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Teleop: navega el robot frente a un AprilTag usando SOLO la camara (Limelight).
 *
 * Dos modos de uso:
 *  1. Auto-detect: new GoToAprilTagCommand(drive, vision)
 *     → Lee el tag que la camara ve en ese momento, busca su grupo en TagGroups,
 *       y usa la distancia configurada para ese grupo.
 *  2. Tag fijo:   new GoToAprilTagCommand(drive, vision, tagId)
 *     → Va directamente a ese tag con la distancia por defecto.
 *
 * No depende del gyro ni de PathPlanner. Control proporcional directo con camara.
 */
public class GoToAprilTagCommand extends Command {

  // Ganancias proporcionales — subir si llega lento, bajar si oscila
  private static final double kP_Distance = 0.6;  // (m/s) por metro de error de distancia
  private static final double kP_Lateral  = 0.4;  // (m/s) por metro de desviacion lateral
  private static final double kP_Rotation = 0.04; // (rad/s) por grado de tx

  // Limites de velocidad
  private static final double kMaxForwardSpeed = 1.2; // m/s adelante/atras
  private static final double kMaxStrafeSpeed  = 0.8; // m/s lateral
  private static final double kMaxRotSpeed     = 1.5; // rad/s

  // Tolerancias para declarar "llegue"
  private static final double kDistanceTolerance = 0.1; // metros adelante/atras
  private static final double kLateralTolerance  = 0.08; // metros izquierda/derecha
  private static final double kTxTolerance       = 2.0;  // grados

  private final DriveSubsystem  m_drive;
  private final VisionSubsystem m_vision;
  private final int             m_requestedTagId;    // -1 = auto-detect
  private final double          m_requestedDistance; // 0 = usar el grupo

  // Resueltos en initialize()
  private int    m_resolvedTagId;
  private double m_resolvedDistance;

  private final Timer m_timer = new Timer();

  /**
   * Modo AUTO-DETECT: lee el tag visible en ese momento, determina su grupo
   * (de TagGroups), y usa la distancia configurada para ese grupo.
   */
  public GoToAprilTagCommand(DriveSubsystem drive, VisionSubsystem vision) {
    this(drive, vision, -1, 0);
  }

  /**
   * Modo tag fijo con distancia por defecto (kDefaultDistanceFromTagMeters).
   */
  public GoToAprilTagCommand(DriveSubsystem drive, VisionSubsystem vision, int tagId) {
    this(drive, vision, tagId, VisionConstants.kDefaultDistanceFromTagMeters);
  }

  /**
   * Modo tag fijo con distancia especificada.
   */
  public GoToAprilTagCommand(DriveSubsystem drive, VisionSubsystem vision, int tagId,
      double distanceMeters) {
    m_drive           = drive;
    m_vision          = vision;
    m_requestedTagId  = tagId;
    m_requestedDistance = distanceMeters;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_resolvedTagId   = -1;
    m_resolvedDistance = 0;

    if (m_requestedTagId == -1) {
      // Modo auto-detect: leer tag de camara y buscar su grupo
      int seenId = m_vision.getTargetId();
      TagGroup group = (seenId >= 0) ? TagGroups.getGroup(seenId) : null;

      if (seenId < 0 || group == null) {
        String reason = seenId < 0 ? "SIN_TAG" : "TAG_" + seenId + "_SIN_GRUPO";
        SmartDashboard.putString("GoToTag/Status", reason);
        System.out.println("[GoToAprilTag] Auto-detect: " + reason + ". Comando abortado.");
        return;
      }

      m_resolvedTagId   = seenId;
      m_resolvedDistance = group.distanceMeters;
      SmartDashboard.putString("GoToTag/Grupo", group.name);
      System.out.println("[GoToAprilTag] Auto-detect: tag=" + seenId
          + " grupo='" + group.name + "' dist=" + group.distanceMeters + "m");

    } else {
      // Modo tag fijo
      m_resolvedTagId   = m_requestedTagId;
      m_resolvedDistance = m_requestedDistance;
      System.out.println("[GoToAprilTag] Tag fijo: " + m_resolvedTagId
          + " dist=" + m_resolvedDistance + "m");
    }

    SmartDashboard.putNumber("GoToTag/TagId",    m_resolvedTagId);
    SmartDashboard.putNumber("GoToTag/DistObj",  m_resolvedDistance);
    SmartDashboard.putString("GoToTag/Status",   "BUSCANDO");
  }

  @Override
  public void execute() {
    if (m_resolvedTagId < 0) return; // no se resolvio en initialize, ya terminara

    if (!m_vision.isSeeingTag(m_resolvedTagId)) {
      m_drive.driveRobotRelative(new ChassisSpeeds());
      SmartDashboard.putString("GoToTag/Status", "TAG_PERDIDO");
      return;
    }

    // Leer los 3 ejes de targetpose_robotspace (metros)
    // [0]=lateral (X), [1]=vertical (Y), [2]=profundidad/distancia (Z)
    double[] targetPose = m_vision.getTargetPoseRobotSpace();
    double distance      = (targetPose.length >= 3) ? Math.abs(targetPose[2]) : 0;
    double lateralOffset = (targetPose.length >= 1) ? targetPose[0] : 0;
    // lateralOffset > 0 → tag a la derecha del robot → strafear derecha (vy negativo en WPILib)
    // Si el robot va al lado contrario, invertir el signo: cambiar -kP_Lateral por +kP_Lateral

    // Error de angulo horizontal: positivo = tag a la derecha
    double tx = m_vision.getTx();

    // Control proporcional en los 3 ejes
    double xSpeed = kP_Distance * (distance - m_resolvedDistance); // adelante/atras
    double ySpeed = -kP_Lateral * lateralOffset;                    // strafe: centra el robot frente al tag
    double omega  = -kP_Rotation * tx;                              // rotacion: apunta al tag

    // Limitar velocidades
    xSpeed = MathUtil.clamp(xSpeed, -kMaxForwardSpeed, kMaxForwardSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -kMaxStrafeSpeed,  kMaxStrafeSpeed);
    omega  = MathUtil.clamp(omega,  -kMaxRotSpeed,     kMaxRotSpeed);

    m_drive.driveRobotRelative(new ChassisSpeeds(xSpeed, ySpeed, omega));

    SmartDashboard.putNumber("GoToTag/Dist_m",      distance);
    SmartDashboard.putNumber("GoToTag/DistErr_m",   distance - m_resolvedDistance);
    SmartDashboard.putNumber("GoToTag/Lateral_m",   lateralOffset);
    SmartDashboard.putNumber("GoToTag/tx_deg",      tx);
    SmartDashboard.putString("GoToTag/Status",      "NAVEGANDO");
  }

  @Override
  public boolean isFinished() {
    // No se resolvio ningun tag: terminar inmediatamente
    if (m_resolvedTagId < 0) return true;

    // Timeout de seguridad
    if (m_timer.get() > VisionConstants.kGoToAprilTagTimeoutSeconds) return true;

    // Verificar si llegamos en los 3 ejes
    if (!m_vision.isSeeingTag(m_resolvedTagId)) return false;
    double[] targetPose  = m_vision.getTargetPoseRobotSpace();
    double distance      = (targetPose.length >= 3) ? Math.abs(targetPose[2]) : 0;
    double lateralOffset = (targetPose.length >= 1) ? targetPose[0] : 0;

    boolean atDistance  = Math.abs(distance - m_resolvedDistance) < kDistanceTolerance;
    boolean atCenter    = Math.abs(lateralOffset) < kLateralTolerance;
    boolean facingTag   = Math.abs(m_vision.getTx()) < kTxTolerance;
    return atDistance && atCenter && facingTag;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.driveRobotRelative(new ChassisSpeeds());
    m_timer.stop();
    String status = interrupted ? "INTERRUMPIDO" : "COMPLETADO";
    SmartDashboard.putString("GoToTag/Status", status);
    System.out.println("[GoToAprilTag] " + status
        + " | tag=" + m_resolvedTagId + " | tiempo=" + m_timer.get() + "s");
  }
}
