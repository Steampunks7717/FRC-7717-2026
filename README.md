# 🤖 FRC Team 7717 - Steam Punks
## 2026 Competition Robot

Repositorio del código del robot para la temporada 2026 de FIRST Robotics Competition.

---

## 📋 Tabla de Contenidos

1. [Información del Equipo](#información-del-equipo)
2. [Requisitos del Sistema](#requisitos-del-sistema)
3. [Configuración Inicial](#configuración-inicial)
4. [Estructura del Proyecto](#estructura-del-proyecto)
5. [Construcción y Despliegue](#construcción-y-despliegue)
6. [Controles](#controles)
7. [Subsistemas](#subsistemas)
8. [Autonomous](#autonomous)
9. [Troubleshooting](#troubleshooting)
10. [Contribución](#contribución)

---

## 👥 Información del Equipo

- **Equipo:** 7717 - Steam Punks
- **Temporada:** 2026
- **Competencia:** FIRST Robotics Competition
- **Repositorio:** [GitHub/FRC-7717-2026]

---

## 🛠️ Requisitos del Sistema

### Software Requerido

- **WPILib Suite** (última versión)
  - Descargar desde: https://github.com/wpilibsuite/allwpilib/releases
- **VS Code** con extensiones WPILib
- **Java JDK 17+**
- **REV Hardware Client** (para configuración de Spark MAX)
- **Git** (para control de versiones)

### Hardware del Robot

- **RoboRIO** (FIRST Robotics Controller)
- **Spark MAX** motor controllers
- **NEO** o **NEO 550** motors
- **Batería 12V**
- **Controladores Xbox/Joystick**

---

## 🚀 Configuración Inicial

### 1. Clonar el Repositorio

```bash
git clone [URL_DEL_REPOSITORIO]
cd FRC-7717-2026
```

### 2. Abrir en VS Code

```bash
code .
```

VS Code debería detectar automáticamente el proyecto WPILib y cargar las extensiones necesarias.

### 3. Configurar WPILib

1. Abrir la paleta de comandos (`Ctrl+Shift+P` / `Cmd+Shift+P`)
2. Ejecutar: `WPILib: Manage Vendor Libraries`
3. Asegurarse de que todas las dependencias estén instaladas

### 4. Verificar Hardware

- Conectar RoboRIO a la red
- Verificar conexiones CAN
- Probar motores con REV Hardware Client

---

## 📁 Estructura del Proyecto

```
FRC-7717-2026/
├── src/
│   ├── main/
│   │   ├── java/
│   │   │   └── frc/
│   │   │       └── robot7717/
│   │   │           ├── Main.java          # Punto de entrada
│   │   │           ├── Robot.java         # Clase principal del robot
│   │   │           ├── RobotContainer.java # Configuración de controles
│   │   │           ├── subsystems/        # Subsistemas del robot
│   │   │           │   ├── DriveSubsystem.java
│   │   │           │   ├── IntakeSubsystem.java
│   │   │           │   ├── ShooterSubsystem.java
│   │   │           │   └── ElevatorSubsystem.java
│   │   │           ├── commands/          # Comandos
│   │   │           │   ├── autonomous/
│   │   │           │   └── ...
│   │   │           └── utils/             # Utilidades
│   │   └── deploy/                         # Archivos de despliegue
├── .wpilib/                                # Configuración WPILib
├── .vscode/                                # Configuración VS Code
├── build.gradle                            # Configuración de build
├── settings.gradle                         # Configuración del proyecto
└── README.md                               # Este archivo
```

---

## 🔨 Construcción y Despliegue

### Compilar el Código

```bash
# Desde VS Code: Ctrl+Shift+P -> "WPILib: Build Robot Code"
# O desde terminal:
./gradlew build
```

### Desplegar al Robot

1. **Conectar al Robot:**
   - Conectar RoboRIO a la red (Ethernet o USB)
   - Verificar conexión en Driver Station

2. **Desplegar Código:**
   - Presionar `F5` en VS Code
   - O usar: `Ctrl+Shift+P` -> "WPILib: Deploy Robot Code"

3. **Verificar Despliegue:**
   - Revisar consola para errores
   - Verificar en Driver Station que el código está corriendo

### Modo Simulación

Para probar sin hardware:

```bash
./gradlew simulateJava
```

---

## 🎮 Controles

### Driver (Controlador Principal)

- **Left Stick Y:** Movimiento adelante/atrás
- **Right Stick X:** Rotación izquierda/derecha
- **Arcade Drive** por defecto
- **Tank Drive** disponible (modificar en RobotContainer)

### Operator (Controlador Secundario)

- **Botones:** Mecanismos específicos
- **Triggers:** Funciones especiales
- Ver `RobotContainer.java` para mapeo completo

### Modificadores

- **Turbo Mode:** Aumenta velocidad máxima
- **Slow Mode:** Reduce velocidad para precisión

---

## ⚙️ Subsistemas

### DriveSubsystem

Sistema de conducción del robot.

- **Motores:** [Especificar cantidad y IDs CAN]
- **Tipo:** Tank/Arcade Drive
- **Encoders:** Integrados en NEO
- **Configuración:** Ver código fuente

### IntakeSubsystem

Sistema de recolección de elementos.

- **Motores:** [Especificar]
- **Sensores:** [Limit switches, etc.]

### ShooterSubsystem

Sistema de disparo.

- **Control PID:** Velocidad constante
- **Motores:** [Especificar]
- **RPM Target:** [Configurar según necesidad]

### ElevatorSubsystem

Sistema de elevación/brazo.

- **Control PID:** Posición
- **Posiciones Preestablecidas:** [Listar]
- **Límites:** Software y hardware

---

## 🤖 Autonomous

### Rutinas Disponibles

1. **Simple Forward**
   - Avanza X metros
   - Se detiene

2. **Place and Exit**
   - Coloca elemento
   - Sale de zona de inicio

3. **Custom Routine**
   - [Describir rutina específica]

### Selección de Autonomous

- Usar Driver Station para seleccionar rutina
- O configurar en `RobotContainer.java`

### Path Planning

- **PathWeaver:** Para rutas complejas
- **PathPlanner:** Alternativa moderna
- Ver documentación WPILib para más detalles

---

## 🔧 Troubleshooting

### Problema: Robot no responde

**Solución:**
1. Verificar conexión de red
2. Verificar que Driver Station está conectado
3. Revisar logs en Driver Station
4. Verificar que código está desplegado

### Problema: Motores no giran

**Solución:**
1. Verificar CAN IDs en código vs hardware
2. Usar REV Hardware Client para diagnosticar
3. Verificar conexiones físicas
4. Revisar límites de corriente

### Problema: Autonomous no funciona

**Solución:**
1. Verificar selección en Driver Station
2. Revisar logs para errores
3. Probar rutina simple primero
4. Verificar encoders/gyro están funcionando

### Problema: Build falla

**Solución:**
1. Ejecutar `./gradlew clean`
2. Verificar que WPILib está actualizado
3. Revisar errores de compilación
4. Verificar dependencias en `build.gradle`

---

## 📚 Recursos de Aprendizaje

### Documentación

- **WPILib Docs:** https://docs.wpilib.org
- **REVLib Docs:** https://docs.revrobotics.com
- **Spark MAX Manual:** https://docs.revrobotics.com/sparkmax/

### Material Educativo del Equipo

- Ver `/FRC/README.md` para guía completa de Spark MAX y motores
- Tutoriales internos en [ubicación]

### Comunidades

- **Chief Delphi:** https://www.chiefdelphi.com
- **FRC Discord:** Comunidad activa
- **Reddit r/FRC:** Discusiones generales

---

## 👨‍💻 Contribución

### Flujo de Trabajo

1. **Crear Branch:**
   ```bash
   git checkout -b feature/nombre-feature
   ```

2. **Hacer Cambios:**
   - Escribir código limpio y comentado
   - Seguir convenciones del equipo
   - Probar antes de commitear

3. **Commit:**
   ```bash
   git add .
   git commit -m "Descripción clara del cambio"
   ```

4. **Push y Pull Request:**
   ```bash
   git push origin feature/nombre-feature
   ```
   - Crear PR en GitHub
   - Esperar revisión antes de merge

### Convenciones de Código

- **Nombres:** camelCase para variables, PascalCase para clases
- **Comentarios:** Explicar "por qué", no "qué"
- **Subsistemas:** Un archivo por subsistema
- **Comandos:** Un comando por acción

### Antes de Merge

- [ ] Código compila sin errores
- [ ] Probado en simulación o hardware
- [ ] Sin warnings críticos
- [ ] Comentarios actualizados
- [ ] Revisado por otro miembro

---

## 📊 Telemetría y Logging

### SmartDashboard

- Datos en tiempo real
- Valores de sensores
- Estado de subsistemas

### Advantage Scope

- Análisis avanzado de datos
- Replay de matches
- Gráficas detalladas

### Data Logging

- Logs automáticos durante matches
- Revisar para debugging
- Ubicación: `/logs/` en RoboRIO

---

## 🏆 Competencia

### Checklist Pre-Competencia

- [ ] Código desplegado y probado
- [ ] Autonomous funcionando
- [ ] Controles mapeados correctamente
- [ ] Telemetría configurada
- [ ] Backup del código
- [ ] Hardware verificado
- [ ] Baterías cargadas
- [ ] Herramientas listas

### Durante la Competencia

- Mantener código en Git actualizado
- Documentar cambios rápidos
- Probar autonomous en práctica
- Ajustar PID si es necesario

---

## 📝 Changelog

### [Versión] - YYYY-MM-DD

#### Agregado
- Nuevas funcionalidades

#### Cambiado
- Modificaciones existentes

#### Corregido
- Bugs resueltos

---

## 📞 Contacto

- **Mentor Principal:** [Nombre/Email]
- **Lead Programmer:** [Nombre/Email]
- **GitHub Issues:** Para reportar bugs
- **Discord/Slack:** Para comunicación rápida

---

## 📄 Licencia

Este proyecto es propiedad de FRC Team 7717 - Steam Punks.

**Uso interno del equipo únicamente.**

---

## 🙏 Agradecimientos

- **WPILib Team** por el framework excelente
- **REV Robotics** por Spark MAX y NEO
- **FIRST** por la competencia
- **Mentores y Padres** por el apoyo
- **Comunidad FRC** por compartir conocimiento

---

## 🎯 Objetivos de la Temporada 2026

- [ ] Robot funcional completo
- [ ] Autonomous confiable
- [ ] Telemetría implementada
- [ ] Código bien documentado
- [ ] Todos los miembros aprenden
- [ ] ¡Diversión y excelencia!

---

## 🚀 Próximos Pasos

1. Configurar hardware específico
2. Implementar mecanismos según diseño
3. Tuning de PID
4. Desarrollar autonomous
5. Práctica y refinamiento

---

**¡Vamos Steam Punks! 🤖⚙️**

*Última actualización: 2026*
*Versión: 1.0.0*
