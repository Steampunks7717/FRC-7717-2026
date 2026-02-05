# Fase 4 — Escalabilidad: preparar más subsistemas

**Objetivo:** Definir cómo se agregan mecanismos (intake, shooter, elevador, etc.) sin desordenar el código ni acoplar todo al chasis. El chasis sigue siendo la base estable; lo demás se suma de forma ordenada.

---

## Prerrequisitos

- Fases 0–3 completadas: chasis documentado, teleop + robot-relative, PathPlanner funcionando, telemetría útil.

---

## Tareas

- [ ] **Convenciones de código**  
  - Un subsistema por mecanismo lógico (ej. IntakeSubsystem, ShooterSubsystem).  
  - Métodos públicos claros (ej. `setSpeed()`, `run()`, `getState()`).  
  - Comandos que usan un subsistema y no duplican lógica; constantes del mecanismo en `Constants` o en una clase dedicada si crece mucho.

- [ ] **Estructura de paquetes**  
  Mantener `subsystems/` para subsistemas. Crear `commands/` (y si se quiere `commands/auto`, `commands/teleop`) para comandos. Cuando se usen **event markers** de PathPlanner, registrar “named commands” que apunten a estos comandos (ej. “intake”, “shoot”) para que el auto los llame en puntos del path.

- [ ] **Chasis como dependencia clara**  
  Los autos y comandos que muevan el robot usan `DriveSubsystem` vía `driveRobotRelative` o `drive`. Ningún otro subsistema debe “poseer” el drive ni reemplazar su default command sin una razón documentada (ej. comando de alineación que temporalmente toma el control).

- [ ] **Ejemplo de integración**  
  Cuando haya al menos un mecanismo nuevo (aunque sea de prueba), tener un comando que lo use y un binding en RobotContainer (botón → comando). El chasis sigue igual; el nuevo subsistema se prueba aislado y luego junto al drive.

---

## Pruebas

- Añadir un subsistema “dummy” o un comando simple (ej. que imprima o mueva un motor de prueba) y verificar que se integra sin tocar el drive.
- Cuando exista un intake/shooter real: un comando que solo use ese subsistema y un botón; el chasis debe seguir respondiendo igual en teleop y en auto.

---

## Criterio de “listo”

- Hay un patrón claro y documentado (en README o en este doc) para agregar subsistemas y comandos.
- El chasis no depende de mecanismos que aún no existan; los mecanismos nuevos dependen del drive solo cuando necesiten mover el robot.

---

## Notas / decisiones del equipo

- 

---

**Anterior:** [Fase 3](PLAN-FASE-3.md) · **Siguiente:** [Fase 5 — Limelight (opcional)](PLAN-FASE-5.md)
