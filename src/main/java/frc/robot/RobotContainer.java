// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SwerveDrive swerveDrive;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    final int frontLeftSpinId = 6;
    final int frontLeftRotateId = 5;
    final int frontRightSpinId = 3;
    final int frontRightRotateId = 4;
    final int backLeftSpinId = 8;
    final int backLeftRotateId = 2;
    final int backRightSpinId = 7;
    final int backRightRotateId = 1;
    final int frontLeftRotateSensorId = 9;
    final int frontRightRotateSensorId = 10;
    final int backLeftRotateSensorId = 11;
    final int backRightRotateSensorId = 12;
    final double frontLeftRotationOffset = 30;
    final double frontRightRotationOffset = 185;
    final double backLeftRotationOffset = 175;
    final double backRightRotationOffset = 105;

    swerveDrive = new SwerveDrive(
            new SwerveModule(
              frontLeftSpinId,
              frontLeftRotateId,
              frontLeftRotateSensorId,
              frontLeftRotationOffset),
            new SwerveModule(
              frontRightSpinId,
              frontRightRotateId,
              frontRightRotateSensorId,
              frontRightRotationOffset),
            new SwerveModule(
              backLeftSpinId,
              backLeftRotateId,
              backLeftRotateSensorId,
              backLeftRotationOffset),
            new SwerveModule(
              backRightSpinId,
              backRightRotateId,
              backRightRotateSensorId,
              backRightRotationOffset));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    swerveDrive.setDefaultCommand(
            new RunCommand(() -> {
              swerveDrive.drive(
                      m_driverController.getLeftX(),
                      m_driverController.getLeftY(),
                      m_driverController.getRightX());
            },
            swerveDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
