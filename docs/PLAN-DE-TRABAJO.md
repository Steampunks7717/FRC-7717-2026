# Plan de trabajo — Chasis y base escalable

Team 7717 Steam Punks · Temporada 2026

Cada **fase** tiene su propio documento para poder debatir y avanzar sin perder contexto.

---

## Índice de fases

| Fase | Documento | Enfoque | Estado |
|------|-----------|--------|--------|
| **0** | [PLAN-FASE-0.md](PLAN-FASE-0.md) | Orden y documentación (base) | Pendiente |
| **1** | [PLAN-FASE-1.md](PLAN-FASE-1.md) | Chasis en teleop al 100% y API robot-relative | **Completada** |
| **2** | [PLAN-FASE-2.md](PLAN-FASE-2.md) | PathPlanner — autos desde la GUI | **Completada (código)** |
| **3** | [PLAN-FASE-3.md](PLAN-FASE-3.md) | Afinar seguimiento y telemetría | Siguiente |
| **4** | [PLAN-FASE-4.md](PLAN-FASE-4.md) | Escalabilidad — preparar más subsistemas | Pendiente |
| **5** | [PLAN-FASE-5.md](PLAN-FASE-5.md) | Limelight (opcional) | Pendiente |

---

## Orden recomendado

```
0 → 1 → 2 → 3 → 4     (5 cuando haya Limelight y necesidad de visión)
```

- **0:** Base documentada y legible.
- **1:** Teleop sólido + métodos que PathPlanner y autos van a usar.
- **2:** Autos definidos en PathPlanner y ejecutados desde el chooser.
- **3:** PID y telemetría para depurar y repetir.
- **4:** Patrón claro para agregar intake, shooter, etc.
- **5:** Visión solo cuando tenga sentido para el juego.

---

## Recursos relacionados

- [INVESTIGACION-FRC-2026.md](INVESTIGACION-FRC-2026.md) — FRC, Swerve, PathPlanner, Advantage Scope, Limelight y análisis del código actual.

---

## Progreso

- **Fase 1 completada:** `getRobotRelativeSpeeds()`, `driveRobotRelative(ChassisSpeeds)` y publicación `/SwerveStates` en DriveSubsystem.
- **Fase 2 completada (código):** PathPlannerLib añadido, AutoBuilder configurado en DriveSubsystem, auto chooser en RobotContainer, deploy pathplanner en `src/main/deploy/pathplanner/`. Falta: configurar robot en PathPlanner GUI, crear path/auto y guardar en deploy.
- **Próximo paso:** Configurar PathPlanner GUI (robot + path + auto) y probar; o Fase 3 (afinar PID/telemetría).

*Para debatir una fase, abre su documento y trabaja sobre ese archivo; el contexto queda guardado ahí.*
