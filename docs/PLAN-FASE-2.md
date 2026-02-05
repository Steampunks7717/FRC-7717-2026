# Fase 2 — PathPlanner: autos desde la GUI

**Estado:** Completada (código). Pendiente: configurar robot en PathPlanner GUI y crear al menos un path/auto.

**Objetivo:** Dejar de tener el auto “hardcodeado” en Java y pasar a definir paths y autos en la app PathPlanner, cargándolos en código con PathPlannerLib (AutoBuilder + PathPlannerAuto).

---

## Prerrequisitos

- **Fase 1 terminada:** existen `getRobotRelativeSpeeds()` y `driveRobotRelative(ChassisSpeeds)` en el DriveSubsystem. Cumplido.

---

## Tareas

- [x] **Añadir PathPlannerLib** al proyecto (`vendordeps/PathplannerLib.json`).

- [ ] **PathPlanner GUI — Robot Config**  
  Configurar el robot con los mismos valores que en `Constants`: posiciones de los módulos (wheelbase, trackwidth), wheel radius, reducción, max speed (m/s). Guardar en la carpeta del proyecto (deploy/pathplanner).

- [ ] **PathPlanner GUI — Path y Auto**  
  Crear al menos un path simple (ej. recta + curva) y un auto que use ese path. Guardar en `src/main/deploy/pathplanner/` (o la ruta que use la app; luego copiar ahí para deploy).

- [x] **DriveSubsystem — AutoBuilder**  
  En el constructor se carga `RobotConfig.fromGUISettings()` (try/catch) y se llama a `AutoBuilder.configure(...)` con getPose, resetOdometry, getRobotRelativeSpeeds, driveRobotRelative, PPHolonomicDriveController(5,0,0 para traslación y rotación), config, supplier “flip for red”, y this.

- [x] **RobotContainer — Auto chooser**  
  Si AutoBuilder está configurado, se construye el chooser con `AutoBuilder.buildAutoChooser()` y se pone en SmartDashboard. En `getAutonomousCommand()` se devuelve el seleccionado; si no hay PathPlanner config, se usa el auto WPILib por defecto.

- [x] **Deploy**  
  `build.gradle` ya despliega `src/main/deploy` al RoboRIO. Creada carpeta `src/main/deploy/pathplanner/` con README; los JSON de PathPlanner deben quedar ahí (o en la estructura pathplanner/autos, pathplanner/paths que genere la GUI).

---

## Pruebas

- **Simulación:** Seleccionar el auto en el chooser, poner el robot en modo auto y ver que el robot sigue el path (aunque sea aproximado en sim).
- **Robot:** Ejecutar el mismo auto en campo; ajustar los PID del `PPHolonomicDriveController` (traslación y rotación) hasta que el seguimiento sea estable y repetible.
- **Alianza roja:** Comprobar que el path se voltea correctamente cuando la alianza es roja (supplier “flip for red”).

---

## Criterio de “listo”

- Un auto definido en PathPlanner se elige desde el chooser y el chasis lo sigue de forma estable.
- El equipo entiende cómo editar paths en la GUI y cómo se cargan en código.

---

## Notas / decisiones del equipo

- Fase 2 código lista. Sin archivos PathPlanner en deploy, el robot usa el auto WPILib por defecto. Tras configurar la GUI y guardar en deploy/pathplanner, hacer deploy y el chooser mostrará los autos.
- Tras clonar o añadir `PathplannerLib.json`: ejecutar Build o “Refresh Gradle” para que la dependencia se descargue y desaparezcan los errores de import en el IDE.

---

**Siguiente:** [Fase 3](PLAN-FASE-3.md) (afinar PID y telemetría) o configurar PathPlanner GUI y probar.

**Anterior:** [Fase 1](PLAN-FASE-1.md)
