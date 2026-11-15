# MAXSwerve Java Template adaptado al equipo 7717

Este repositorio es el [REV MAXSwerve Java Template](https://github.com/REVrobotics/MAXSwerve-Java-Template) v2026.0 con los valores de nuestro chasis. El primer commit es el template sin modificar; el segundo aplica solo estos cambios:

| Archivo | Cambio | Template | 7717 |
|---|---|---|---|
| `.wpilib/wpilib_preferences.json` | `teamNumber` | 2714 | 7717 |
| `Constants.java` (`DriveConstants`) | `kMaxSpeedMetersPerSecond` | 4.8 | 0.3 |
| `Constants.java` (`DriveConstants`) | `kMaxAngularSpeed` | `2 * Math.PI` | `0.25 * Math.PI` |
| `Constants.java` (`DriveConstants`) | `kTrackWidth` y `kWheelBase` | 26.5 in | 24.5 in |
| `Constants.java` (`DriveConstants`) | CAN IDs de drive (FL, RL, FR, RR) | 11, 13, 15, 17 | 4, 6, 2, 8 |
| `Constants.java` (`DriveConstants`) | CAN IDs de giro (FL, RL, FR, RR) | 10, 12, 14, 16 | 3, 5, 1, 7 |
| `RobotContainer.java` | último argumento de `drive(...)` en el comando por defecto | `true` (relativo al campo) | `false` (relativo al robot) |

Lo que sigue igual y hay que revisar: el template usa un `ADIS16470_IMU` como giroscopio, y nuestro robot anterior usaba un `ADXRS450_Gyro`. Por eso el movimiento queda relativo al robot hasta que se cambie el giroscopio.

Los valores de CAN IDs, dimensiones y velocidades salen del código del robot 2026 del equipo y no se han probado en este chasis. La velocidad máxima está baja a propósito.

El resto de este archivo es el README original del template.

---

# MAXSwerve Java Template v2026.0

See [the online changelog](https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/CHANGELOG.md) for information about updates to the template that may have been released since you created your project.

## Description

A template project for an FRC swerve drivetrain that uses REV MAXSwerve Modules.

Note that this template is designed for a drivetrain composed of four MAXSwerve Modules, each configured with two SPARKS MAX, a NEO as the driving motor, a NEO 550 as the turning motor, and a REV Through Bore Encoder as the absolute turning encoder. If you are using SPARK Flex for either the drive motor or turning motor, you will need to update the classes accordingly.

To get started, make sure you have calibrated the zero offsets for the absolute encoders in Hardware Client 2 using the `Absolute Encoder` utility under the associated turning SPARK devices.

## Prerequisites

* SPARK MAX Firmware v26.1.0
* REVLib v2026.0.0

## Configuration

It is possible that this project will not work for your robot right out of the box. Various things like the CAN IDs, PIDF gains, chassis configuration, etc. must be determined for your own robot!

These values can be adjusted in the `Configs.java` and `Constants.java` files.
