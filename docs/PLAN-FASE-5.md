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

- [x] **Lectura de Limelight**  
  VisionSubsystem lee directamente de NetworkTables (tabla `limelight`): tx, ty, tv, tid. No se usa LimelightHelpers; se puede añadir después si se necesita más API.

- [x] **VisionSubsystem**  
  Subsistema que expone `getTx()`, `getTy()`, `hasTarget()`, `getTargetId()`, `getTagPoseFromLayout(id)`, `getTargetPoseInFrontOfTag(id, distance)` y `isSeeingTag(id)`. Layout de tags 9 y 10 en Constants (VisionConstants).

- [x] **Comando “ir al AprilTag” (teleop)**  
  `GoToAprilTagCommand`: al pulsar botón (A = tag 9, B = tag 10) el robot se coloca centrado frente al tag a distancia configurable. Usa trayectoria para movimiento rápido; timeout 5 s; si no ve el tag en 1 s termina sin bloquear; al soltar el botón el comando sigue hasta llegar o timeout (no se cancela).

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

- Botones: A = AprilTag 9, B = AprilTag 10. Constantes en `OIConstants.kGoToAprilTag9Button` y `kGoToAprilTag10Button`.
- Poses de tags 9 y 10 en `VisionConstants.kTag9FieldPose` y `kTag10FieldPose`; actualizar desde el manual del juego.
- Opcional: feedback cuando no hay tag (vibración, LED).

---

**Anterior:** [Fase 4](PLAN-FASE-4.md) · **Índice:** [Plan de trabajo](PLAN-DE-TRABAJO.md)
