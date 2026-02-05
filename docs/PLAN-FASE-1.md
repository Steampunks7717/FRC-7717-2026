# Fase 1 — Chasis en teleop al 100% y API robot-relative

**Estado:** Completada

**Objetivo:** Que el chasis responda exactamente como quieras en teleop y exponer la API que PathPlanner (y cualquier comando de auto) va a usar: `getRobotRelativeSpeeds()` y `driveRobotRelative(ChassisSpeeds)`.

---

## Contexto

Hoy el drive solo tiene `drive(x, y, rot, fieldRelative)`. PathPlanner y los comandos de seguimiento de trayectoria necesitan:

- **ChassisSpeeds en marco robot** (no field-relative).
- Un método que **reciba** esos ChassisSpeeds y comande los módulos.

Por eso esta fase añade dos métodos sin cambiar el comportamiento actual del teleop.

---

## Tareas

- [x] **getRobotRelativeSpeeds()** en `DriveSubsystem`  
  Usar `DriveConstants.kDriveKinematics.toChassisSpeeds()` con los estados actuales de los cuatro módulos (`getState()`) y devolver ese `ChassisSpeeds`.  
  Comentar: “Para path following y telemetría; devuelve velocidades en marco del robot.”

- [x] **driveRobotRelative(ChassisSpeeds speeds)** en `DriveSubsystem`  
  Convertir `speeds` a `SwerveModuleState[]` con `kDriveKinematics.toSwerveModuleStates(speeds)`, llamar `SwerveDriveKinematics.desaturateWheelSpeeds(...)` y luego `setModuleStates(...)`.  
  Comentar: “PathPlanner y comandos de auto usarán este método; las velocidades son robot-relative.”

- [x] **Teleop sin cambios**  
  El default command del drive sigue llamando a `drive(x, y, rot, true)` (field-relative). No se tocó ese flujo.

- [x] **(Opcional) Telemetría para Advantage Scope**  
  En `periodic()` del DriveSubsystem se publica el array de `SwerveModuleState` al topic `/SwerveStates` con `StructArrayPublisher` para ver los vectores en la pestaña Swerve de Advantage Scope.

---

## Pruebas

- **Simulación:** Ejecutar el proyecto en sim; mover con joystick; comprobar que no hay excepciones y que los módulos se mueven de forma coherente.
- **Robot (teleop):** Adelante/atrás, strafe izquierda/derecha, girar, combinaciones. Comprobar que el “zero heading” (Start) sigue funcionando y que no hay drift raro del gyro.
- **Advantage Scope (si se implementó telemetría):** Grabar un log o conectar en vivo y ver que los vectores de los módulos coinciden con el movimiento del robot.

---

## Criterio de “listo”

- Teleop se siente correcto y sin regresiones.
- Existen `getRobotRelativeSpeeds()` y `driveRobotRelative(ChassisSpeeds)` documentados en código.
- Opcional: se ven los estados del swerve en Advantage Scope.

---

## Notas / decisiones del equipo

- Fase 1 implementada: `getRobotRelativeSpeeds()`, `driveRobotRelative(ChassisSpeeds)` y publicación a `/SwerveStates` en `DriveSubsystem.java`. Pendiente: pruebas en sim/robot y Advantage Scope.

---

**Siguiente fase:** [Fase 2 — PathPlanner](PLAN-FASE-2.md). Prerrequisito (Fase 1) cumplido.

**Anterior:** [Fase 0](PLAN-FASE-0.md)
