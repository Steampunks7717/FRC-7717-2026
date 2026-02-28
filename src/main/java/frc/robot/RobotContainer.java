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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** Starting position chooser (1, 2, or 3). Alliance is read from DriverStation automatically. */
  private final SendableChooser<Integer> m_positionChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Named commands must be registered BEFORE buildAutoChooser so PathPlanner can find them.
    NamedCommands.registerCommand("Shoot",
        Commands.run(() -> m_shooter.shoot(), m_shooter)
            .withTimeout(10)
            .andThen(new InstantCommand(() -> m_shooter.shooterStop(), m_shooter)));

    // Starting position chooser: driver selects 1, 2, or 3 on SmartDashboard.
    // Alliance (Red/Blue) is read automatically from the DriverStation at match start.
    m_positionChooser.setDefaultOption("Posicion 1", 1);
    m_positionChooser.addOption("Posicion 2", 2);
    m_positionChooser.addOption("Posicion 3", 3);
    SmartDashboard.putData("Posicion Inicio", m_positionChooser);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    // Teleop: one button = position robot centered in front of AprilTag (fast, with timeout)
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new GoToAprilTagCommand(m_robotDrive, m_vision, 9));
    new JoystickButton(m_driverController,  XboxController.Button.kB.value)
        .onTrue(new GoToAprilTagCommand(m_robotDrive, m_vision, 10));
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(() -> m_shooter.shoot(), m_shooter));
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> m_intake.intake1(), m_intake));

  }
     

  /**
   * Builds the autonomous command based on:
   *  - Alliance: read automatically from DriverStation (set by field/DS).
   *  - Starting position: chosen by the driver on SmartDashboard (1, 2, or 3).
   *
   * Auto file must exist in deploy/pathplanner/autos/ with the matching name,
   * e.g. "Blue1.auto", "Red2.auto", "Blue3.auto".
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
      System.out.println("[Auto] Auto '" + autoName + "' no encontrado en PathPlanner. Sin auto.");
      SmartDashboard.putString("Auto/Seleccionado", autoName + " - NO ENCONTRADO");
      return new InstantCommand();
    }
  }

}
