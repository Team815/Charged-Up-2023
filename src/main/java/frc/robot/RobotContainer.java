// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController m_driverController =
        new CommandXboxController(0);

    private final SwerveDrive swerveDrive;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
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
        final double frontLeftAngularOffset = -140;
        final double frontRightAngularOffset = 72;
        final double backLeftAngularOffset = 98;
        final double backRightAngularOffset = 171.5;

        swerveDrive = new SwerveDrive(
            new SwerveModule(
                frontLeftSpinId,
                frontLeftRotateId,
                frontLeftRotateSensorId,
                frontLeftAngularOffset),
            new SwerveModule(
                frontRightSpinId,
                frontRightRotateId,
                frontRightRotateSensorId,
                frontRightAngularOffset),
            new SwerveModule(
                backLeftSpinId,
                backLeftRotateId,
                backLeftRotateSensorId,
                backLeftAngularOffset),
            new SwerveModule(
                backRightSpinId,
                backRightRotateId,
                backRightRotateSensorId,
                backRightAngularOffset));

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
        m_driverController.start().onTrue(new InstantCommand(swerveDrive::resetGyro));
        m_driverController.rightBumper().onTrue(new InstantCommand(swerveDrive::resetPose));


        // The robot assumes positive vertical direction is forward,
        // but the controller positive vertical direction is down (backward).
        // Therefore, we must negate the left joystick's Y direction.
        //
        // The robot also assumes positive sideways direction is to the left,
        // but the controller positive sideways direction is to the right.
        // Therefore, we must negate the left joystick's X direction.
        swerveDrive.setDefaultCommand(
            new RunCommand(() -> {
                final double deadband = 0.15;
                swerveDrive.drive(
                    MathUtil.applyDeadband(-m_driverController.getLeftY(), deadband),
                    MathUtil.applyDeadband(-m_driverController.getLeftX(), deadband),
                    MathUtil.applyDeadband(m_driverController.getRightX(), deadband));
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
        return swerveDrive.driveHeart();
    }
}
