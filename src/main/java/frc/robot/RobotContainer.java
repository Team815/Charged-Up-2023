// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.Commander;
import frc.robot.input.InputDevice;
import frc.robot.input.XboxController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDriveModule;
import frc.robot.subsystems.SwerveModule;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private InputDevice inputDevice;
    private final SwerveDrive swerveDrive;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        final var frontLeftSpinId = 6;
        final var frontLeftRotateId = 5;
        final var frontRightSpinId = 3;
        final var frontRightRotateId = 4;
        final var backLeftSpinId = 8;
        final var backLeftRotateId = 2;
        final var backRightSpinId = 7;
        final var backRightRotateId = 1;
        final var frontLeftRotateSensorId = 9;
        final var frontRightRotateSensorId = 10;
        final var backLeftRotateSensorId = 11;
        final var backRightRotateSensorId = 12;
        final var frontLeftAngularOffset = -173d;
        final var frontRightAngularOffset = -35d;
        final var backLeftAngularOffset = 95d;
        final var backRightAngularOffset = 170d;

        var moduleFrontLeft = new SwerveModule(
            frontLeftSpinId,
            frontLeftRotateId,
            frontLeftRotateSensorId,
            frontLeftAngularOffset);

        var moduleFrontRight = new SwerveModule(
            frontRightSpinId,
            frontRightRotateId,
            frontRightRotateSensorId,
            frontRightAngularOffset);

        var moduleBackLeft = new SwerveModule(
            backLeftSpinId,
            backLeftRotateId,
            backLeftRotateSensorId,
            backLeftAngularOffset);

        var moduleBackRight = new SwerveModule(
            backRightSpinId,
            backRightRotateId,
            backRightRotateSensorId,
            backRightAngularOffset);

        final var halfLength = 0.368d;
        final var halfWidth = 0.368d;

        swerveDrive = new SwerveDrive(
            new Pigeon2(0),
            new SwerveDriveModule(moduleFrontLeft, halfLength, halfWidth),
            new SwerveDriveModule(moduleFrontRight, halfLength, -halfWidth),
            new SwerveDriveModule(moduleBackLeft, -halfLength, halfWidth),
            new SwerveDriveModule(moduleBackRight, -halfLength, -halfWidth));

        inputDevice = new XboxController();
        configureBindings();
    }

    private void configureBindings() {
        var commander = new Commander(swerveDrive);

        inputDevice.resetHeading().onTrue(commander.resetGyro(0));

        inputDevice.slow().whileTrue(new StartEndCommand(
            () -> {
                inputDevice.setMaxSidewaysSpeed(0.2d);
                inputDevice.setMaxForwardSpeed(0.2d);
                inputDevice.setMaxAngularSpeed(0.2d);
            },
            () -> {
                inputDevice.setMaxSidewaysSpeed(1d);
                inputDevice.setMaxForwardSpeed(1d);
                inputDevice.setMaxAngularSpeed(1d);
            }));

        inputDevice.driveTo().onTrue(commander.driveTo(20, -10, 0));

        swerveDrive.setDefaultCommand(commander.accelerationDrive(
            () -> inputDevice.getForwardVelocity(),
            () -> inputDevice.getSidewaysVelocity(),
            () -> inputDevice.getAngularVelocity()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new PrintCommand("Autonomous");
    }

    public void setInputDevice(InputDevice inputDevice) {
        this.inputDevice = inputDevice;
        configureBindings();
    }
}
