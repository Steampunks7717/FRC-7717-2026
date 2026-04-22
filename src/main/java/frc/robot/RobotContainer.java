// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.GoToAprilTagCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem(m_robotDrive);

  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** Position chooser (1/2/3) combined with alliance color to select the autonomous routine. */
  private final SendableChooser<Integer> m_positionChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("Shooter", Commands.run(() -> m_shooter.shootauto(), m_shooter).withTimeout(10).andThen(new InstantCommand(() -> m_shooter.shooterStop(), m_shooter)));
    NamedCommands.registerCommand("Intake", Commands.run(() -> m_intake.intake1(), m_intake).withTimeout(3).andThen(new InstantCommand(() -> m_intake.intakeStop(), m_intake)));

    m_positionChooser.setDefaultOption("Posicion 1", 1);
    m_positionChooser.addOption("Posicion 2", 2);
    m_positionChooser.addOption("Posicion 3", 3);
    SmartDashboard.putData("Posicion Inicio", m_positionChooser);

    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  private void configureButtonBindings() {
    // RB: lock wheels in X formation (anti-tipping)
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // Start: reset gyro heading (re-orient field-relative)
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // Right Trigger: shoot + feed intake
    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.2)
        .onTrue(new RunCommand(() -> m_shooter.shoot(), m_shooter))
        .onFalse(new RunCommand(() -> m_shooter.shooterStop(), m_shooter));
    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.2)
        .onTrue(new RunCommand(() -> m_intake.intake1(), m_intake))
        .onFalse(new RunCommand(() -> m_intake.intakeStop(), m_intake));

    // LB: reverse shooter + reverse intake
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(new RunCommand(() -> m_shooter.shootrevert(), m_shooter))
        .onFalse(new RunCommand(() -> m_shooter.shooterStop(), m_shooter));
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(new RunCommand(() -> m_intake.intake1(), m_intake))
        .onFalse(new RunCommand(() -> m_intake.intakeStop(), m_intake));

    // Left Trigger: pass shot + intake
    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.2)
        .onTrue(new RunCommand(() -> m_intake.intake1(), m_intake))
        .onFalse(new RunCommand(() -> m_intake.intakeStop(), m_intake));
    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.2)
        .onTrue(new RunCommand(() -> m_shooter.shooterpass(), m_shooter))
        .onFalse(new RunCommand(() -> m_shooter.shooterStop(), m_shooter));

    // X: auto-shoot
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(new RunCommand(() -> m_shooter.shootauto(), m_shooter))
        .onFalse(new RunCommand(() -> m_shooter.shooterStop(), m_shooter));

    // Y: climb up  |  A: climb down
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new RunCommand(() -> m_climber.ClimbUp(), m_climber))
        .onFalse(new RunCommand(() -> m_climber.ClimberStop(), m_climber));
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new RunCommand(() -> m_climber.ClimbDown(), m_climber))
        .onFalse(new RunCommand(() -> m_climber.ClimberStop(), m_climber));
  }

  /**
   * Returns the autonomous command selected by alliance color + starting position.
   * Auto name format: "Blue1", "Blue2", "Blue3", "Red1", "Red2", "Red3".
   * Falls back to "auto1" if the named auto is not found.
   */
  public Command getAutonomousCommand() {
    if (!AutoBuilder.isConfigured()) {
      System.out.println("[Auto] PathPlanner no configurado. Sin auto.");
      return new InstantCommand();
    }

    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    String allianceName = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
        ? "Red" : "Blue";
    int position = m_positionChooser.getSelected();
    String autoName = allianceName + position; // e.g. "Blue2", "Red3"

    System.out.println("[Auto] Alianza=" + allianceName + " | Posicion=" + position
        + " | Cargando auto: " + autoName);
    SmartDashboard.putString("Auto/Seleccionado", autoName);

    try {
      return AutoBuilder.buildAuto(autoName);
    } catch (Exception e) {
      System.out.println("[Auto] Auto '" + autoName + "' no encontrado. Usando 'auto1' como respaldo.");
      SmartDashboard.putString("Auto/Seleccionado", autoName + " -> auto1 (respaldo)");
      try {
        return AutoBuilder.buildAuto("auto1");
      } catch (Exception e2) {
        System.out.println("[Auto] 'auto1' tampoco encontrado. Sin auto.");
        SmartDashboard.putString("Auto/Seleccionado", "NINGUNO");
        return new InstantCommand();
      }
    }
  }
}
