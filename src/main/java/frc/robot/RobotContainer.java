// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.GoToAprilTagCommand;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.List;

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

  // private final Climber m_climber = new Climber();


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** PathPlanner auto chooser; null when PathPlanner config is not present. */
  private final SendableChooser<Integer> m_positionChooser = new SendableChooser<>();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("Shooter", Commands.run(() -> m_shooter.shootauto(), m_shooter).withTimeout(10).andThen(new InstantCommand(() -> m_shooter.shooterStop(), m_shooter)));
    NamedCommands.registerCommand("Intake", Commands.run(() -> m_intake.intake1(), m_intake).withTimeout(3).andThen(new InstantCommand(() -> m_intake.intakeStop(), m_intake)));
    // PathPlanner: build chooser when AutoBuilder was configured (has RobotConfig from GUI)
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
    //  new JoystickButton(m_driverController, XboxController.Button.kA.value)
    //     .onTrue(new GoToAprilTagCommand(m_robotDrive, m_vision, 9));
    // new JoystickButton(m_driverController,  XboxController.Button.kB.value)
    //     .onTrue(new GoToAprilTagCommand(m_robotDrive, m_vision, 10));;
     new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.2).onTrue(new RunCommand(() -> m_shooter.shoot(),m_shooter)).onFalse(new RunCommand(() -> m_shooter.shooterStop(), m_shooter)); 
    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.2).onTrue(new RunCommand(() -> m_intake.intake1(), m_intake)).onFalse(new RunCommand(() -> m_intake.intakeStop(), m_intake));    // new JoystickButton(m_driverController, XboxController.Button.kX.value).onTrue(new RunCommand(() -> m_shooter.travel(), m_shooter)).onFalse(new RunCommand(() -> m_shooter.shooterStop(), m_shooter));
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).onTrue(new RunCommand(() -> m_shooter.shootrevert(), m_shooter)).onFalse(new RunCommand(() -> m_shooter.shooterStop(), m_shooter));
      new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).onTrue(new RunCommand(() -> m_intake.intake1(), m_intake)).onFalse(new RunCommand(() -> m_intake.intakeStop(), m_intake));
    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.2).onTrue(new RunCommand(() -> m_intake.intake1(), m_intake)).onFalse(new RunCommand(() -> m_intake.intakeStop(), m_intake));    // new JoystickButton(m_driverController, XboxController.Button.kX.value).onTrue(new RunCommand(() -> m_shooter.travel(), m_shooter)).onFalse(new RunCommand(() -> m_shooter.shooterStop(), m_shooter));
    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.2).onTrue(new RunCommand(() -> m_shooter.shooterpass(), m_shooter)).onFalse(new RunCommand(() -> m_shooter.shooterStop(), m_shooter));
    new JoystickButton(m_driverController, XboxController.Button.kX.value).onTrue(new RunCommand(() -> m_shooter.shootauto(), m_shooter)).onFalse(new RunCommand(() -> m_shooter.shooterStop(), m_shooter));

    new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(new RunCommand(() -> m_climber.ClimbUp(), m_climber)).onFalse(new RunCommand(() -> m_climber.ClimberStop(), m_climber));
    new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(new RunCommand(() -> m_climber.ClimbDown(), m_climber)).onFalse(new RunCommand(() -> m_climber.ClimberStop(), m_climber));
   
    // new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(new RunCommand(() -> m_climber.ClimbUp(), m_climber)).onFalse(new RunCommand(() -> m_climber.ClimberStop(), m_climber));
    // new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(new RunCommand(() -> m_climber.ClimbDown(), m_climber)).onFalse(new RunCommand(() -> m_climber.ClimberStop(), m_climber));
   
    // new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(new RunCommand(() -> m_shooter.shootrevert(), m_shooter)).onFalse(new RunCommand(() -> m_shooter.shooterStop(), m_shooter));
  }
     

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * When PathPlanner is configured, returns the selected auto from the chooser.
   * Otherwise returns a no-op command (only PathPlanner autos are run).
   * 
   *
   * @return the command to run in autonomous
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
        + " | Cargando auto:   +" + autoName);
    SmartDashboard.putString("Auto/Seleccionado", autoName);

    try {
      // return AutoBuilder.buildAuto(autoName);
        return AutoBuilder.buildAuto("Blue2");
    } catch (Exception e) {
            // Alliance+position auto not found — fall back to "auto1" (default test auto)
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





  // if (!AutoBuilder.isConfigured()) {
  //     System.out.println("[Auto] PathPlanner no configurado. Sin auto.");
  //     return new InstantCommand();
  //   }

  //   Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
  //   String allianceName = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
  //       ? "Red" : "Blue";
  //   int position = m_positionChooser.getSelected();
  //   String autoName = allianceName + position; // e.g. "Blue2", "Red3"

  //   System.out.println("[Auto] Alianza=" + allianceName + " | Posicion=" + position
  //       + " | Cargando auto: " + autoName);
  //   SmartDashboard.putString("Auto/Seleccionado", autoName);

  //   try {
  //     // return AutoBuilder.buildAuto(autoName);
  //       return AutoBuilder.buildAuto("Blue2");
  //   } catch (Exception e) {
  //           // Alliance+position auto not found — fall back to "auto1" (default test auto)
  //     System.out.println("[Auto] Auto '" + autoName + "' no encontrado. Usando 'auto1' como respaldo.");
  //     SmartDashboard.putString("Auto/Seleccionado", autoName + " -> auto1 (respaldo)");
  //     try {
  //       return AutoBuilder.buildAuto("auto1");
  //     } catch (Exception e2) {
  //       System.out.println("[Auto] 'auto1' tampoco encontrado. Sin auto.");
  //       SmartDashboard.putString("Auto/Seleccionado", "NINGUNO");
  //       return new InstantCommand();
  //     }
  // }





  // if (!AutoBuilder.isConfigured()) {
  //     System.out.println("[Auto] PathPlanner no configurado. Sin auto.");
  //     return new InstantCommand();
  //   }

  //   Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
  //   String allianceName = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
  //       ? "Red" : "Blue";
  //   int position = m_positionChooser.getSelected();
  //   String autoName = allianceName + position; // e.g. "Blue2", "Red3"

  //   System.out.println("[Auto] Alianza=" + allianceName + " | Posicion=" + position
  //       + " | Cargando auto: " + autoName);
  //   SmartDashboard.putString("Auto/Seleccionado", autoName);

  //   try {
  //     // return AutoBuilder.buildAuto(autoName);
  //       return AutoBuilder.buildAuto("Blue2");
  //   } catch (Exception e) {
  //           // Alliance+position auto not found — fall back to "auto1" (default test auto)
  //     System.out.println("[Auto] Auto '" + autoName + "' no encontrado. Usando 'auto1' como respaldo.");
  //     SmartDashboard.putString("Auto/Seleccionado", autoName + " -> auto1 (respaldo)");
  //     try {
  //       return AutoBuilder.buildAuto("auto1");
  //     } catch (Exception e2) {
  //       System.out.println("[Auto] 'auto1' tampoco encontrado. Sin auto.");
  //       SmartDashboard.putString("Auto/Seleccionado", "NINGUNO");
  //       return new InstantCommand();
  //     }
  // }




  
    // TrajectoryConfig config = new TrajectoryConfig(
    //   AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
    //   AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //   .setKinematics(DriveConstants.kDriveKinematics);

    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(4,0, new Rotation2d(0)),
    //   List.of(), 
    //   new Pose2d(0,0, new Rotation2d(0)),
    //   config);

    // var thetaController = new ProfiledPIDController(
    //   AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints); 
    //   thetaController.enableContinuousInput(-Math.PI,Math.PI);

    // SwerveControllerCommand swerveControllerCommand  = new SwerveControllerCommand(
    //   trajectory,
    //   m_robotDrive::getPose, 
    //   DriveConstants.kDriveKinematics, 
    //   new PIDController(AutoConstants.kPXController,0,0), 
    //   new PIDController(AutoConstants.kPYController,0,0), 
    //   thetaController, 
    //   m_robotDrive::setModuleStates, 
    //   m_robotDrive);

    // m_robotDrive.resetOdometry(trajectory.getInitialPose());

    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));