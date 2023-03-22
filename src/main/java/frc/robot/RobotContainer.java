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
    private final GamePieceLimelight limelight;

    static {
        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("ScoreCross", 0);
        autonChooser.addOption("ScoreCrossLevelRight", 1);
        autonChooser.addOption("ScoreCrossLevelLeft", 2);
        autonChooser.addOption("ScoreCrossLevelCenter", 3);
        autonChooser.addOption("Test", 4);
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

        swerveDrive = new SwerveDrive(moduleFrontLeft, moduleFrontRight, moduleBackLeft, moduleBackRight);

        limelight = new GamePieceLimelight("limelight-field");

        inputDevice = new XboxController();

        // Shuffleboard config tab

        Dashboard.createSwerveModuleLayout(configTab, 0, 0, moduleFrontLeft, moduleFrontRight, moduleBackLeft, moduleBackRight);
        Dashboard.createSwerveDriveLayout(configTab, 0, 2, swerveDrive);
        Dashboard.createControllerLayout(configTab, 2, 0, () -> inputDevice, this);

        // Shuffleboard readings tab

        Dashboard.createPoseLayout(readingsTab, 0, 0, swerveDrive::getPose);
        Dashboard.createVelocityLayout(readingsTab, 0, 2, swerveDrive::getSpeeds);
        Dashboard.createAnglesLayout(readingsTab, 2, 0, swerveDrive::getAngles);
        Dashboard.createLimelightLayout(readingsTab, 2, 2, limelight);

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
        var commander = new RobotCommander(swerveDrive);

        //Gyro
        inputDevice.resetHeading().onTrue(commander.resetGyro(0));

        //Limelight
        inputDevice.cycleLimelightTarget().onTrue(new InstantCommand(limelight::cycleTarget));

        //Center Target
        inputDevice.centerOnTarget().whileTrue(new CenterOnTarget(
            swerveDrive,
            limelight::getHorizontalOffset,
            () -> inputDevice.getForwardVelocity(),
            () -> inputDevice.getSidewaysVelocity(),
            0.02d,
            1d));

        swerveDrive.setDefaultCommand(
            new RunCommand(() -> swerveDrive.drive(
                inputDevice.getForwardVelocity(),
                inputDevice.getSidewaysVelocity(),
                inputDevice.getAngularVelocity()),
                swerveDrive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        var commander = new RobotCommander(swerveDrive);
        var autonIndex = autonChooser.getSelected();
        var auton = autonIndex == 1 ? Autos.scoreCrossLevelRight(commander)
            : autonIndex == 2 ? Autos.scoreCrossLevelLeft(commander)
            : autonIndex == 3 ? Autos.scoreCrossLevelCenter(commander)
            : autonIndex == 4 ? Autos.test(commander)
            : Autos.scoreCross(commander);
        return auton;
    }

    public void setInputDevice(InputDevice inputDevice) {
        this.inputDevice = inputDevice;
        configureBindings();
    }
}
