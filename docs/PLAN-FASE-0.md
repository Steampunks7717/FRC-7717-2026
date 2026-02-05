# Fase 0 — Orden y documentación (base)

**Objetivo:** Dejar claro qué hace cada parte del chasis y cómo se entiende el código. Sin esto, cualquier cambio posterior es más difícil de debatir y mantener.

---

## Tareas

- [ ] **Constants.java** — Revisar que cada grupo (DriveConstants, ModuleConstants, OIConstants, AutoConstants, NeoMotorConstants) tenga un comentario de bloque explicando para qué sirve y en qué unidades están los valores (metros, rad/s, CAN IDs, etc.).
- [ ] **DriveSubsystem.java** — Añadir comentario de clase (qué es el subsistema, qué controla) y en cada método público: qué recibe, qué hace y qué devuelve (o qué efecto tiene).
- [ ] **MAXSwerveModule.java** — Comentario de clase (qué es un módulo MAXSwerve, drive + turn) y en métodos públicos (getState, getPosition, setDesiredState, resetEncoders) una línea que explique el propósito.
- [ ] **Configs.java** — Comentario de clase indicando que son configuraciones REV para los SPARK MAX del MAXSwerve (drive y turn), y por qué se usan estos factores de conversión / PID.
- [ ] **RobotContainer.java** — Comentario breve en `configureButtonBindings()` indicando qué botón hace qué (R1 = setX, Start = zero heading) y en `getAutonomousCommand()` qué tipo de auto se está usando (ejemplo WPILib por ahora).
- [ ] **README.md** — Actualizar para que describa solo lo que existe: swerve (4× MAXSwerve), controles (Xbox, field-relative, setX, zero heading), auto de ejemplo WPILib. Añadir enlace a `docs/INVESTIGACION-FRC-2026.md` para contexto y a este plan (`docs/PLAN-DE-TRABAJO.md`).

---

## Pruebas

No hay pruebas de robot en esta fase. El criterio es de **lectura**: alguien que abra el repo entiende la estructura y el flujo del chasis sin tener que adivinar.

- Revisión de código: otro miembro (o tú en unos días) lee los archivos y confirma que los comentarios aclaran el “por qué” y las unidades.

---

## Criterio de “listo”

- Constantes, subsistemas y RobotContainer tienen comentarios que explican propósito y uso.
- README refleja el estado real del proyecto (solo drive swerve) y apunta a la investigación y al plan de trabajo.

---

## Notas / decisiones del equipo

*(Espacio para anotar acuerdos, dudas resueltas o cambios respecto a este documento.)*

- 

---

**Siguiente:** [Fase 1 — Chasis teleop y robot-relative](PLAN-FASE-1.md)
