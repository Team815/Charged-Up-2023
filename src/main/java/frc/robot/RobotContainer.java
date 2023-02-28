// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.input.InputDevice;
import frc.robot.input.Joystick;
import frc.robot.input.XboxController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.EnumSet;
import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private InputDevice inputDevice;
    private static final GenericEntry inputDeviceChoiceEntry;
    private static double maxTeleopXSpeed;
    private static double maxTeleopYSpeed;
    private static double maxTeleopAngularSpeed;
    private static final GenericEntry maxTeleopXSpeedEntry;
    private static final GenericEntry maxTeleopYSpeedEntry;
    private static final GenericEntry maxTeleopAngularSpeedEntry;

    private final SwerveDrive swerveDrive;

    static {
        maxTeleopXSpeed = 1d;
        maxTeleopYSpeed = 1d;
        maxTeleopAngularSpeed = 1d;
        var tab = Shuffleboard.getTab("SmartDashboard");
        var layout = tab
            .getLayout("Teleop", BuiltInLayouts.kGrid)
            .withSize(2, 2)
            .withProperties(Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 4));
        inputDeviceChoiceEntry = layout
            .add("Joystick <-> Xbox Controller", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(0, 0)
            .getEntry();
        maxTeleopXSpeedEntry = layout
            .add("Max Vertical Speed", maxTeleopXSpeed)
            .withPosition(0, 1)
            .getEntry();
        maxTeleopYSpeedEntry = layout
            .add("Max Horizontal Speed", maxTeleopYSpeed)
            .withPosition(0, 2)
            .getEntry();
        maxTeleopAngularSpeedEntry = layout
            .add("Max Angular Speed", maxTeleopAngularSpeed)
            .withPosition(0, 3)
            .getEntry();
    }

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
        final var frontLeftAngularOffset = -140d;
        final var frontRightAngularOffset = 72d;
        final var backLeftAngularOffset = 98d;
        final var backRightAngularOffset = 171.5d;

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

        inputDevice = new XboxController();

        var inst = NetworkTableInstance.getDefault();
        inst.addListener(
            inputDeviceChoiceEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> {
                var useXboxController = e.valueData.value.getBoolean();
                inputDevice = useXboxController ? new XboxController() : new Joystick();
            });
        inst.addListener(
            maxTeleopXSpeedEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> maxTeleopXSpeed = e.valueData.value.getDouble());
        inst.addListener(
            maxTeleopYSpeedEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> maxTeleopYSpeed = e.valueData.value.getDouble());
        inst.addListener(
            maxTeleopAngularSpeedEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> maxTeleopAngularSpeed = e.valueData.value.getDouble());

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
        inputDevice.resetHeading().onTrue(new InstantCommand(swerveDrive::resetGyro));
        inputDevice.toggleLimelightTarget().onTrue(new InstantCommand(swerveDrive::toggleLimelightTarget));


        // The robot assumes positive vertical direction is forward,
        // but the controller positive vertical direction is down (backward).
        // Therefore, we must negate the left joystick's Y direction.
        //
        // The robot also assumes positive sideways direction is to the left,
        // but the controller positive sideways direction is to the right.
        // Therefore, we must negate the left joystick's X direction.
        swerveDrive.setDefaultCommand(
            new RunCommand(() -> swerveDrive.drive(
                inputDevice.getVerticalSpeed() * maxTeleopXSpeed,
                inputDevice.getHorizontalSpeed() * maxTeleopYSpeed,
                inputDevice.getAngularSpeed() * maxTeleopAngularSpeed,
                0.5d),
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
