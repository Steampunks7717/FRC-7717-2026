# Fase 3 — Afinar seguimiento y telemetría

**Objetivo:** Mejorar el seguimiento de paths (PID y límites) y tener datos claros para depurar y explicar el comportamiento del chasis en cualquier momento.

---

## Prerrequisitos

- Fase 2 terminada: autos de PathPlanner se ejecutan desde el chooser.

---

## Tareas

- [ ] **Ajustar PID del path follower**  
  Probar en campo el auto de PathPlanner y afinar los PID de traslación (x, y) y de rotación (theta) del `PPHolonomicDriveController`. Documentar los valores finales (en `Constants` o en comentarios junto a `AutoBuilder.configure`).

- [ ] **Field2d (opcional)**  
  Si PathPlanner o el código permiten callbacks de “path activo”, publicar la pose actual y/o la trayectoria a un widget Field2d en Shuffleboard para ver en tiempo real la posición del robot y el path.

- [ ] **Mantener telemetría de swerve**  
  Si en Fase 1 se publicaron `SwerveModuleState[]` (y ChassisSpeeds) a NetworkTables, seguir usándolos. Revisar en Advantage Scope que los vectores tienen sentido durante auto y teleop.

- [ ] **Repetibilidad**  
  Ejecutar el mismo auto varias veces y anotar si el robot termina en posiciones similares; si hay mucha dispersión, revisar odometría, zero heading y posibles patinajes.

---

## Pruebas

- Grabar logs con auto y teleop; abrirlos en Advantage Scope y revisar timeline, pestaña Swerve y (si aplica) pose en el campo.
- Repetir el mismo auto al menos 3–5 veces y verificar que el comportamiento es consistente.

---

## Criterio de “listo”

- El seguimiento del path es estable y los PID están documentados.
- Hay telemetría suficiente (Swerve en Advantage Scope y opcionalmente Field2d) para explicar qué hace el chasis en cada momento.

---

## Notas / decisiones del equipo

- 

---

**Anterior:** [Fase 2](PLAN-FASE-2.md) · **Siguiente:** [Fase 4](PLAN-FASE-4.md)
