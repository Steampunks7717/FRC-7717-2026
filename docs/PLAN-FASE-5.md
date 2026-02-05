# Fase 5 — Limelight (opcional)

**Objetivo:** Usar Limelight para targeting (alineación con objetivo) o para mejorar la pose del robot (AprilTag / MegaTag2), sin atar el diseño del chasis a la visión. El drive sigue recibiendo ChassisSpeeds o setpoints; la visión solo genera esos setpoints o corrige la pose.

---

## Cuándo hacer esta fase

- Cuando el equipo tenga **hardware Limelight** y una necesidad clara: por ejemplo alinear para disparar o anotar, o mejorar la localización en auto con AprilTags.
- No es obligatoria para tener el chasis “al 100%”; el chasis puede estar completo sin visión.

---

## Prerrequisitos

- Fase 1 al menos: `getPose()` y `resetOdometry()` disponibles; si se usa pose estimation, conviene tener el drive estable (Fase 2–3).

---

## Tareas

- [ ] **LimelightHelpers**  
  Añadir el archivo `LimelightHelpers.java` al proyecto (repo oficial Limelight o documentación) para leer datos de la cámara vía NetworkTables.

- [ ] **VisionSubsystem (o integración en drive)**  
  Crear un subsistema (o métodos en un subsistema existente) que lean de Limelight: `getTX()`, `getTY()`, `getTV()` para targeting; opcionalmente pose con `getBotPoseEstimate_wpiBlue_MegaTag2()` si se usa AprilTag.

- [ ] **Comandos que usan visión**  
  Por ejemplo “AlinearseConObjetivo”: lee tx/ty (y tv), aplica un PID que genera rotación (y opcionalmente avance) y comanda el drive con `drive()` o `driveRobotRelative()`. El drive no sabe que la referencia viene de Limelight; solo recibe ChassisSpeeds o x, y, rot.

- [ ] **Pose estimation (opcional)**  
  Si se quiere corregir la pose con AprilTags: usar `SwerveDrivePoseEstimator` (WPILib). En `periodic()` del drive (o del vision), llamar `addVisionMeasurement(pose, timestamp)` cuando Limelight devuelva una pose válida (MegaTag2). El `getPose()` que usa PathPlanner sigue siendo el del drive; la visión solo corrige.

- [ ] **Configuración en Limelight**  
  Acceder a la web de la cámara (`http://limelight.local:5801`), configurar pipeline(s), exposición y (si aplica) uso de AprilTag para pose.

---

## Pruebas

- **Targeting:** Con cámara apuntando a un objetivo (retroreflector o AprilTag), ver que tx/ty se actualizan en SmartDashboard o en código. Activar el comando de alineación y comprobar que el robot acerca tx a 0 (y ty si se usa).
- **Pose estimation (si aplica):** Con AprilTags visibles, ver en Field2d (o en Advantage Scope) que la pose estimada se acerca a la real y que el auto no se desorienta al pasar por tags.

---

## Criterio de “listo”

- Limelight está integrada y hay al menos un comando que usa visión (targeting o pose).
- El chasis no tiene lógica específica de Limelight; solo recibe setpoints generados por comandos que leen la cámara.

---

## Notas / decisiones del equipo

- 

---

**Anterior:** [Fase 4](PLAN-FASE-4.md) · **Índice:** [Plan de trabajo](PLAN-DE-TRABAJO.md)
