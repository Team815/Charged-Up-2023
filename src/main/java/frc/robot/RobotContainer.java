// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.CenterOnTarget;
import frc.robot.input.InputDevice;
import frc.robot.input.XboxController;
import frc.robot.subsystems.SwerveDrive;
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
    private final GamePieceLimelight limelight;

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
        final var frontLeftAngularOffset = -160d;
        final var frontRightAngularOffset = -5d;
        final var backLeftAngularOffset = 98d;
        final var backRightAngularOffset = 171.5d;

        var configTab = "Config";
        var readingsTab = "Readings";

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

        Dashboard.createSwerveModuleLayout(configTab, 0, 0, moduleFrontLeft, moduleFrontRight, moduleBackLeft, moduleBackRight);

        swerveDrive = new SwerveDrive(moduleFrontLeft, moduleFrontRight, moduleBackLeft, moduleBackRight);
        Dashboard.createSwerveDriveLayout(configTab, 0, 2, swerveDrive);
        Dashboard.createPoseLayout(readingsTab, 0, 0, swerveDrive::getPose);
        Dashboard.createVelocityLayout(readingsTab, 0, 2, swerveDrive::getSpeeds);

        limelight = new GamePieceLimelight("limelight-field");
        Dashboard.createLimelightLayout(readingsTab, 2, 0, limelight);

        inputDevice = new XboxController();
        Dashboard.createControllerLayout(configTab, 2, 0, () -> inputDevice, this);

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
        inputDevice.resetHeading().onTrue(new InstantCommand(() -> swerveDrive.resetGyro(0)));
        inputDevice.cycleLimelightTarget().onTrue(new InstantCommand(limelight::cycleTarget));
        inputDevice.centerOnTarget().whileTrue(new CenterOnTarget(
            swerveDrive,
            limelight::getHorizontalOffset,
            () -> inputDevice.getForwardVelocity(),
            () -> inputDevice.getSidewaysVelocity(),
            0.02d));

        swerveDrive.setDefaultCommand(
            new RunCommand(() -> swerveDrive.drive(
                inputDevice.getForwardVelocity(),
                inputDevice.getSidewaysVelocity(),
                inputDevice.getAngularVelocity(),
                0.5d),
                swerveDrive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //return new RunCommand(() -> swerveDrive.drive(0.1, 0, 0, 0.5), swerveDrive);
        // An example command will be run in autonomous
        return Autos.driveOverChargeStation(swerveDrive);
    }

    public void setInputDevice(InputDevice inputDevice) {
        this.inputDevice = inputDevice;
        configureBindings();
    }
}
