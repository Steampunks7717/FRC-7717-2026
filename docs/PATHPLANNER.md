# PathPlanner en el chasis 7717

Con esta configuración, el robot sigue en autónomo las trayectorias que dibujes en PathPlanner. En medio de un autónomo puede llamar al comando de la cámara, que lo acerca a 1 m del AprilTag que vea, y después sigue con lo que venga.

## Qué está configurado

| Pieza | Dónde | Qué hace |
|---|---|---|
| PathPlannerLib 2026.1.2 | `vendordeps/PathplannerLib.json` | La librería oficial, instalada desde su URL de vendordep |
| `AutoBuilder` | `DriveSubsystem` (constructor) | Conecta PathPlanner con la odometría y el swerve |
| `driveRobotRelative` y `getRobotRelativeSpeeds` | `DriveSubsystem` | Mueven el swerve y leen su velocidad, como pide PathPlanner |
| Comando `AcercarseATag` | `RobotContainer` | El comando de la tecla Y, con un límite de 5 s, disponible en PathPlanner |
| `Auto Chooser` | Dashboard | Lista todos los autónomos de `src/main/deploy/pathplanner/autos` |
| Configuración del robot | `src/main/deploy/pathplanner/settings.json` | Medidas y motores del chasis |
| Ejemplo | `paths/Avanzar1m.path`, `autos/AvanzarYAcercarse.auto` | Avanza 1 m y después se acerca al tag |

## Configuración del robot (`settings.json`)

| Dato | Valor | De dónde sale |
|---|---|---|
| Módulos | ±0.311 m del centro | `kTrackWidth` y `kWheelBase` (24.5 in) |
| Radio de rueda | 0.0381 m | `kWheelDiameterMeters` / 2 |
| Reducción de drive | 4.714 | `kDrivingMotorReduction` con piñón de 14 dientes |
| Velocidad máxima física | 4.8 m/s | Velocidad libre del NEO con esa reducción |
| Motor y límite de corriente | NEO, 50 A | `Configs.java` |
| Límites por defecto de las trayectorias | 0.3 m/s, 0.5 m/s², 45 °/s, 90 °/s² | Los mismos límites seguros del teleop |
| Masa, inercia, tamaño del cuadro | 50 kg, 6.0 kg·m², 0.75 m | **Estimados: mídelos y corrígelos** |

El swerve nunca pasa de `kMaxSpeedMetersPerSecond` (0.3 m/s), ni siquiera en autónomo. Si una trayectoria pide más, el robot se queda atrás del plan, y como PathPlanner termina la trayectoria cuando se acaba su tiempo, el robot se detiene antes del punto final. Para ir más rápido, sube los dos valores juntos: `kMaxSpeedMetersPerSecond` en `Constants.java` y los límites de la trayectoria en PathPlanner.

## Cómo usarlo

1. **Instalar PathPlanner** desde [pathplanner.dev](https://pathplanner.dev).
2. **Abrir el proyecto:** en PathPlanner, abre la carpeta de este repo. Lee y guarda en `src/main/deploy/pathplanner`.
3. **Revisar la configuración del robot** en la pantalla de ajustes de PathPlanner y corregir masa, inercia y tamaño.
4. **Dibujar una trayectoria:** crea un path y arrastra sus puntos. Cada path arranca y termina donde lo pongas, con la orientación que le des.
5. **Armar un autónomo:** crea un auto y encadena bloques. Para acercarse al tag, agrega un bloque de comando con nombre y elige `AcercarseATag`. El nombre debe ser exactamente ese. Puedes poner más paths después: cuando el comando termina, el autónomo sigue.
6. **Desplegar:** `WPILib: Deploy Robot Code`. Los archivos de `src/main/deploy` se copian al robot, y los que borraste también se borran del robot (`deleteOldFiles = true`).
7. **Elegir y correr:** en el dashboard, elige el autónomo en `Auto Chooser`. En la Driver Station, pon modo **Autonomous** y habilita, con el robot en el piso y la zona despejada.

## Cosas a saber antes de la primera prueba

- **Posición inicial:** el ejemplo tiene `resetOdom` activado, así que al empezar el robot cree que está donde arranca el primer path, en (2, 4) a 0°. Pon el robot en el piso mirando hacia donde quieres que sea "adelante" del campo.
- **Alianza roja:** `kFlipForRedAlliance` está en `false`. La Driver Station suele tener una estación roja seleccionada, y con el espejo activado las trayectorias se reflejarían al otro lado del campo. En competencia, cámbialo a `true`.
- **`AcercarseATag` necesita ver un tag.** Si no ve ninguno, espera hasta 5 s y el autónomo sigue con el siguiente bloque. La distancia de 1 m se mide desde el lente, y el robot centra el tag en la mira de la cámara: si la cámara no está centrada en el robot, el robot queda desplazado hacia ese lado.
- **Giroscopio:** PathPlanner sigue la trayectoria con la odometría, y la odometría usa el giroscopio. El template usa un `ADIS16470_IMU`. Si el chasis tiene otro, el robot no sabrá cuánto giró y las trayectorias con giro saldrán mal. Las rectas sin giro funcionan mejor.
- **Ganancias:** `kTranslationP` y `kRotationP` en `PathPlannerConstants` valen 5.0, como el ejemplo oficial. Si el robot oscila al seguir la trayectoria, bájalas.

## Si algo falla

| Síntoma | Qué revisar |
|---|---|
| `Auto Chooser` solo muestra `Ninguno` | La consola de la Driver Station dice `PathPlanner no se pudo configurar`: falta o está mal `settings.json` |
| El autónomo no hace nada | ¿Elegiste el auto en el dashboard? ¿Está en modo Autonomous? |
| El robot se desvía o gira de más | Giroscopio, la posición inicial y las ganancias de PathPlanner |
| `AcercarseATag` no se mueve | La cámara no ve tags, o el comando de la tecla Y tampoco funciona en teleop |
| Va más lento que el plan | Los límites de la trayectoria superan los 0.3 m/s del chasis |
