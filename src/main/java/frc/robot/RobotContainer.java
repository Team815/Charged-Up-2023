// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.input.InputDevice;
import frc.robot.input.XboxController;
import frc.robot.subsystems.*;

import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private InputDevice inputDevice;
    private static final SendableChooser<Integer> autonChooser;
    private final SwerveDrive swerveDrive;
    private final Claw claw;
    private final GamePieceLimelight limelight;
    private final Arm arm;
    private final Shoulder shoulder;
    private final Dashboard shuffleboard;

    static {
        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("ScoreCross", 0);
        autonChooser.addOption("ScoreCrossLevelRight", 1);
        autonChooser.addOption("ScoreCrossLevelLeft", 2);
        autonChooser.addOption("Test", 3);
        var tab = Shuffleboard.getTab("SmartDashboard");
        var layout = tab
            .getLayout("Robot", BuiltInLayouts.kGrid)
            .withSize(3, 2)
            .withProperties(Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 1));
        layout
            .add("Autonomous", autonChooser)
            .withPosition(0, 0)
            .withWidget(BuiltInWidgets.kComboBoxChooser);
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
        final var frontLeftAngularOffset = -169d;
        final var frontRightAngularOffset = 112d;
        final var backLeftAngularOffset = -146.5d;
        final var backRightAngularOffset = -11.5d;

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

        swerveDrive = new SwerveDrive(moduleFrontLeft, moduleFrontRight, moduleBackLeft, moduleBackRight);


        final var compressorPort = 30;
        final var solenoidChannel = 0;

        claw = new Claw(
            new Compressor(compressorPort, PneumaticsModuleType.CTREPCM),
            new Solenoid(compressorPort, PneumaticsModuleType.CTREPCM, solenoidChannel)
        );

        limelight = new GamePieceLimelight("limelight-field");
        inputDevice = new XboxController();

        var verticalMotorMainId = 9;
        var verticalMotorSecondaryId = 10;
        var horizontalMotorId = 3;

        arm = new Arm(verticalMotorMainId, verticalMotorSecondaryId);
        shoulder = new Shoulder(horizontalMotorId);

        shuffleboard = new Dashboard(
            "SmartDashboard",
            swerveDrive,
            limelight,
            () -> inputDevice,
            this,
            moduleFrontLeft,
            moduleFrontRight,
            moduleBackLeft,
            moduleBackRight);

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
        //Gyro
        inputDevice.resetHeading().onTrue(new InstantCommand(() -> swerveDrive.resetGyro(0)));

        //Limelight
        inputDevice.cycleLimelightTarget().onTrue(new InstantCommand(limelight::cycleTarget));

        //Center Target
        inputDevice.centerOnTarget().whileTrue(new CenterOnTarget(
            swerveDrive,
            limelight::getHorizontalOffset,
            () -> inputDevice.getForwardVelocity(),
            () -> inputDevice.getSidewaysVelocity(),
            0.02d,
            inputDevice.setArmToStationPickup().getAsBoolean() ? 0.05d : 1d));

        //Claw
        inputDevice.openClaw().whileTrue(
            new MoveShoulder(shoulder, 17000)
                .andThen(new InstantCommand(claw::open)));

        inputDevice.openClaw().onFalse(
            (new InstantCommand(claw::close)
                .alongWith(new WaitCommand(0.5d)))
                .andThen(new MoveShoulder(shoulder, 0d)
                    .alongWith(new KeepArmAt(arm, KeepArmAt.AboveFloor, 0.2d))));

        //Arm
        inputDevice.setArmToTopCone().whileTrue(
            new LiftArmTo(arm, KeepArmAt.FarConeNode, 0.2d)
                .andThen(new MoveShoulder(shoulder, 17000d)
                    .alongWith(new KeepArmAt(arm, KeepArmAt.FarConeNode, 0.2d))));

        inputDevice.setArmToTopCone().onFalse(
            (new WaitCommand(0.5d)
                .deadlineWith(new InstantCommand(claw::open), new KeepArmAt(arm, KeepArmAt.FarConeNode, 0.2d)))
                .andThen(new MoveShoulder(shoulder, 0d)
                    .deadlineWith(new InstantCommand(claw::close), new KeepArmAt(arm, KeepArmAt.FarConeNode, 0.2d)))
                .andThen(new DropArm(arm)));

        inputDevice.setArmToBottomCone().whileTrue(
            new LiftArmTo(arm, KeepArmAt.NearConeNode, 0.2d)
                .andThen(new MoveShoulder(shoulder, 1000d)
                    .alongWith(new KeepArmAt(arm, KeepArmAt.NearConeNode, 0.2d))));

        inputDevice.setArmToBottomCone().onFalse(
            (new WaitCommand(0.5d)
                .deadlineWith(new InstantCommand(claw::open), new KeepArmAt(arm, KeepArmAt.NearConeNode, 0.2d)))
                .andThen(new MoveShoulder(shoulder, 0d)
                    .deadlineWith(new InstantCommand(claw::close), new KeepArmAt(arm, KeepArmAt.NearConeNode, 0.2d)))
                .andThen(new DropArm(arm)));

        inputDevice.setArmToStationPickup().whileTrue(
            new LiftArmTo(arm, KeepArmAt.Substation, 0.2d)
                .andThen(new MoveShoulder(shoulder, 15000)
                    .alongWith(new KeepArmAt(arm, KeepArmAt.Substation, 0.2d), new InstantCommand(claw::open))));

        inputDevice.setArmToStationPickup().onFalse(
            new WaitCommand(0.3d)
                .deadlineWith(new InstantCommand(claw::close), new KeepArmAt(arm, KeepArmAt.Substation, 0.2d))
                .andThen(new MoveShoulder(shoulder, 0d)
                    .deadlineWith(new KeepArmAt(arm, KeepArmAt.Substation, 0.2d)))
                .andThen(new KeepArmAt(arm, KeepArmAt.AboveFloor, 0.1d)));

        inputDevice.turtle().onTrue(new DropArm(arm)
            .alongWith(new MoveShoulder(shoulder, 0d)));

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
        var autonIndex = autonChooser.getSelected();
        var auton = autonIndex == 1 ? Autos.scoreCrossLevelRight(swerveDrive, shoulder, arm, claw)
            : autonIndex == 2 ? Autos.scoreCrossLevelLeft(swerveDrive, shoulder, arm, claw)
            : autonIndex == 3 ? Autos.test(swerveDrive)
            : Autos.scoreCross(swerveDrive, shoulder, arm, claw);
        return new InstantCommand(arm::zeroEncoder)
            .andThen(auton);
    }

    public void setInputDevice(InputDevice inputDevice) {
        this.inputDevice = inputDevice;
        configureBindings();
    }
}
