# AdvantageScope en el chasis 7717

Con esta configuración, el robot publica en vivo todo lo que importa del chasis, la cámara y PathPlanner, y además lo graba en un log. AdvantageScope lo muestra mientras el robot corre y también después, abriendo el log.

## Qué publica el robot

| Tema | Tipo | Qué es |
|---|---|---|
| `Telemetry/Drive/Pose` | `Pose2d` | Dónde cree el robot que está, según la odometría |
| `Telemetry/Drive/Heading` | `Rotation2d` | Ángulo del giroscopio |
| `Telemetry/Drive/ModuleStates/Measured` | `SwerveModuleState[]` | Velocidad y ángulo real de cada módulo |
| `Telemetry/Drive/ModuleStates/Setpoints` | `SwerveModuleState[]` | Lo que se le pidió a cada módulo |
| `Telemetry/Drive/ChassisSpeeds` | `ChassisSpeeds` | Velocidad del robot, relativa al robot |
| `Telemetry/Drive/TurnRateDegPerSec` | número | Velocidad de giro del giroscopio |
| `Telemetry/Robot/BatteryVolts` | número | Voltaje de la batería |
| `Telemetry/Vision/HasTarget`, `TagId` | booleano, número | Si la Limelight ve un tag y cuál |
| `Telemetry/Vision/BotPose` | `Pose2d` | Dónde cree la Limelight que está el robot (`botpose_wpiblue`) |
| `Telemetry/PathPlanner/CurrentPose`, `TargetPose` | `Pose2d` | Dónde está el robot y dónde debería estar en la trayectoria |
| `Telemetry/PathPlanner/ActivePath` | `Pose2d[]` | La trayectoria que está siguiendo |
| `URCL/…` | varios | Cada Spark MAX: corriente, velocidad, voltaje y más |
| `SmartDashboard/ApproachTag/…` | varios | Estado del comando que acerca el robot al tag |

Los módulos van en este orden: delantero izquierdo, delantero derecho, trasero izquierdo y trasero derecho. Las poses usan el sistema WPILib azul.

Los Spark MAX aparecen con nombre: `DriveFrontLeft`, `TurnFrontLeft` y así para los cuatro módulos.

## Instalar

AdvantageScope viene con el instalador de WPILib. También se puede bajar desde sus [releases en GitHub](https://github.com/Mechanical-Advantage/AdvantageScope/releases/latest).

## Ver el robot en vivo

1. En AdvantageScope, abre las preferencias: `App` > `Show Preferences...` en Windows, o `AdvantageScope` > `Settings...` en Mac.
2. Pon la dirección del robot: `10.77.17.2`.
3. Conéctate con `File` > `Connect to Robot`. Mientras busca, el título de la ventana dice "Searching".
4. Con el robot conectado por cable USB, usa antes `File` > `Use USB roboRIO Address` (`172.22.11.2`).

## Grabar y bajar los logs

El robot graba siempre, desde que arranca: todo lo de NetworkTables, la Driver Station y los controles.

- **Dónde:** en una memoria USB conectada al roboRIO, dentro de una carpeta `logs`; si no hay memoria, en `/home/lvuser/logs`. La memoria debe estar en FAT32, y WPILib recomienda de 32 GB o menos.
- **Limpieza automática:** WPILib borra los logs más viejos si queda menos de 50 MB libres, y al arrancar borra los logs a los que nunca se conectó una Driver Station.
- **Bajarlos:** en AdvantageScope, `File` > `Download Logs...`. Los más nuevos salen arriba. Selecciónalos y descárgalos con la flecha.
- **Abrirlos:** abre el archivo `.wpilog` en AdvantageScope. En el log, los temas de NetworkTables aparecen con el prefijo `NT:`.

## Qué mirar en cada pestaña

Para agregar un dato, arrástralo del árbol de la izquierda a la pestaña. Cada pestaña tiene un botón `?` con los tipos de objeto que acepta.

| Pestaña | Arrastra | Para qué |
|---|---|---|
| **2D Field** | `Drive/Pose`, `PathPlanner/TargetPose`, `PathPlanner/ActivePath`, `Vision/BotPose` | Ver el robot en el campo, la trayectoria y dónde lo ubica la cámara |
| **3D Field** | Las mismas poses | Lo mismo en 3D. Se puede cargar un modelo del robot como recurso propio |
| **Swerve** | `ModuleStates/Measured`, `ModuleStates/Setpoints`, `ChassisSpeeds`, `Heading` | Ver cada módulo como una flecha. Ajusta el tamaño del cuadro a 0.62 × 0.62 m y la velocidad máxima a 4.8 m/s |
| **Line Graph** | `BatteryVolts`, `TurnRateDegPerSec`, `URCL/…`, `ApproachTag/DistanciaM` | Ver cómo cambian los números en el tiempo |
| **Joysticks** | Los datos de la Driver Station (en los logs) | Ver qué apretó el piloto en cada momento |
| **Table** | Cualquier dato | Ver los valores exactos en cada instante |
| **Statistics** | Por ejemplo, `BatteryVolts` | Ver mínimos, máximos y distribución |
| **Console** | La salida de consola del robot | Ver errores y mensajes, como el aviso de PathPlanner |
| **Video** | El video del partido o de la prueba | Sincronizarlo con los datos |

## Recetas

- **¿Sigue bien la trayectoria?** En 2D Field, pon `PathPlanner/CurrentPose` y `PathPlanner/TargetPose`. Si se separan, revisa las ganancias de PathPlanner y el giroscopio.
- **¿Hay un módulo flojo o invertido?** En Swerve, compara `Measured` contra `Setpoints`. Un módulo cuya flecha real no sigue a la pedida es el sospechoso.
- **¿La cámara y la odometría coinciden?** En 2D Field, pon `Drive/Pose` y `Vision/BotPose`. Si la cámara salta, mira `Vision/TagId` y la distancia al tag.
- **¿Se cae la batería?** En Line Graph, pon `BatteryVolts` junto a la corriente de los motores de `URCL/…`.
- **¿Qué pasó en la prueba?** Baja el log, ábrelo, carga el video en la pestaña Video y recorre el tiempo con la línea de tiempo.

## Límites

- **Giroscopio:** el template usa un `ADIS16470_IMU`. Si el chasis tiene otro, `Heading` y `TurnRateDegPerSec` se leerán en 0, y la pose girará mal.
- **URCL:** muchas señales de los Spark MAX están apagadas hasta que el código las lee. Algunas pueden faltar en el log hasta que se activen.
- **Sin replay:** esto es registro y visualización con WPILib. Repetir una prueba completa en la computadora requiere AdvantageKit, que no está en este PR.
