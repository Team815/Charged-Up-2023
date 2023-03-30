// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.input.InputDevice;
import frc.robot.input.XboxController;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private InputDevice inputDevice;
    private final SwerveDrive swerveDrive;
    private final Claw claw;
    private final GamePieceLimelight limelight;
    private final Arm arm;
    private final Shoulder shoulder;
    private int autonId;

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

        final var compressorPort = 30;
        final var solenoidChannel = 0;

        claw = new Claw(
            new Compressor(compressorPort, PneumaticsModuleType.CTREPCM),
            new Solenoid(compressorPort, PneumaticsModuleType.CTREPCM, solenoidChannel)
        );

        limelight = new GamePieceLimelight("limelight-field");

        inputDevice = new XboxController();

        var armMotor1Id = 9;
        var armMotor2Id = 10;
        var shoulderMotorId = 22;
        var shoulderEncoderChannel = 1;

        arm = new Arm(armMotor1Id, armMotor2Id);
        shoulder = new Shoulder(shoulderMotorId, shoulderEncoderChannel);

        autonId = 0;

        // Shuffleboard config tab

        Dashboard.createSwerveModuleLayout(configTab, 0, 0, moduleFrontLeft, moduleFrontRight, moduleBackLeft, moduleBackRight);
        Dashboard.createSwerveDriveLayout(configTab, 0, 2, swerveDrive);
        Dashboard.createControllerLayout(configTab, 2, 0, () -> inputDevice, this);

        // Shuffleboard readings tab

        Dashboard.createPoseLayout(readingsTab, 0, 0, swerveDrive::getPose);
        Dashboard.createVelocityLayout(readingsTab, 0, 2, swerveDrive::getSpeeds);
        Dashboard.createAnglesLayout(readingsTab, 2, 0, swerveDrive::getAngles);
        Dashboard.createLimelightLayout(readingsTab, 2, 2, limelight);
        Dashboard.createShoulderLayout(readingsTab, 2, 3, shoulder::getPosition);
        Dashboard.createArmLayout(readingsTab, 4, 0, arm);
        Dashboard.createAutonomousLayout(readingsTab, 6, 0, this);
        Dashboard.createClawLayout(readingsTab, 4, 1, claw);

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
        var commander = new RobotCommander(swerveDrive, shoulder, arm, claw);

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
            inputDevice.setArmToStationPickup().getAsBoolean() ? 0.05d : 1d));

        //Claw
        inputDevice.openClaw().whileTrue(
            commander.moveShoulder(MoveShoulder.Position.PICKUP)
                .andThen(commander.openClaw()));

        inputDevice.openClaw().onFalse(
            commander.closeClaw()
                .alongWith(new WaitCommand(0.5d))
                .andThen(commander.moveShoulder(MoveShoulder.Position.RETRACTED)
                    .alongWith(commander.keepArmAt(KeepArmAt.AboveFloor, KeepArmAt.ConeGroundFf))));

        //Arm
        inputDevice.setArmToTopCone().whileTrue(
            commander.liftArmTo(KeepArmAt.FarConeNode, KeepArmAt.FarConeNodeFf)
                .andThen(commander.moveShoulder(MoveShoulder.Position.FAR_CONE)
                    .alongWith(commander.keepArmAt(KeepArmAt.FarConeNode, KeepArmAt.FarConeNodeFf))));

        inputDevice.setArmToTopCone().onFalse(
            new WaitCommand(0.5d)
                .deadlineWith(
                    commander.openClaw(),
                    commander.keepArmAt(KeepArmAt.FarConeNode, KeepArmAt.NoConeFf))
                .andThen(
                    new WaitUntilCommand(() -> shoulder.getPosition() >= -1d)
                        .deadlineWith(
                            commander.moveShoulder(MoveShoulder.Position.RETRACTED),
                            commander.closeClaw(),
                            commander.keepArmAt(KeepArmAt.FarConeNode, KeepArmAt.NoConeFf)),
                    commander.moveShoulder(MoveShoulder.Position.RETRACTED)
                        .alongWith(commander.dropArm())));

        inputDevice.setArmToBottomCone().whileTrue(
            commander.liftArmTo(KeepArmAt.NearConeNode, KeepArmAt.NearConeNodeFf)
                .andThen(commander.moveShoulder(MoveShoulder.Position.NEAR_CONE)
                    .alongWith(commander.keepArmAt(KeepArmAt.NearConeNode, KeepArmAt.NearConeNodeFf))));

        inputDevice.setArmToBottomCone().onFalse(
            new WaitCommand(0.5d)
                .deadlineWith(
                    commander.openClaw(),
                    commander.keepArmAt(KeepArmAt.NearConeNode, KeepArmAt.NoConeFf))
                .andThen(
                    commander.moveShoulder(MoveShoulder.Position.RETRACTED)
                        .deadlineWith(
                            commander.closeClaw(),
                            commander.keepArmAt(KeepArmAt.NearConeNode, KeepArmAt.NoConeFf)),
                    commander.dropArm()));

        inputDevice.setArmToStationPickup().onTrue(
            commander.liftArmTo(KeepArmAt.Substation, KeepArmAt.NoConeFf)
                .andThen(
                    commander.keepArmAt(KeepArmAt.Substation, KeepArmAt.NoConeFf)
                        .alongWith(
                            new WaitCommand(0.5d)
                                .deadlineWith(commander.openClaw())
                                .andThen(
                                    new WaitUntilCommand(claw::isDetecting),
                                    commander.closeClaw()))));

        inputDevice.setArmToStationPickup().onFalse(
            new WaitCommand(0.3d)
                .deadlineWith(commander.closeClaw())
                .andThen(commander.keepArmAt(KeepArmAt.AboveFloor, 0.15d, KeepArmAt.ConeGroundFf)));

        inputDevice.turtle().onTrue(
            commander.dropArm()
                .alongWith(commander.moveShoulder(MoveShoulder.Position.RETRACTED))
                .andThen(commander.resetShoulder()
                ));

        inputDevice.slow().whileTrue(new StartEndCommand(
            () -> {
                inputDevice.setMaxSidewaysSpeed(0.3d);
                inputDevice.setMaxForwardSpeed(0.3d);
                inputDevice.setMaxAngularSpeed(0.3d);
            },
            () -> {
                inputDevice.setMaxSidewaysSpeed(1d);
                inputDevice.setMaxForwardSpeed(1d);
                inputDevice.setMaxAngularSpeed(1d);
            }));

        inputDevice.test1().onTrue(new InstantCommand(() -> arm.set(arm.getPower() + 0.01d)));
        inputDevice.test2().onTrue(new InstantCommand(() -> arm.set(arm.getPower() - 0.01d)));

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
        var commander = new RobotCommander(swerveDrive, shoulder, arm, claw);
        var auton = autonId == 0 ? Autos.scoreCrossLevelCenter(commander)
            : autonId == 1 ? Autos.scoreCrossLevelRight(commander)
            : autonId == 2 ? Autos.scoreCrossLevelLeft(commander)
            : autonId == 3 ? Autos.test(commander)
            : Autos.scoreCrossLevelCenter(commander);
        return auton.withTimeout(14.8d).andThen(new RunCommand(() -> swerveDrive.drive(0d, 0d, 0d)));
    }

    public void setInputDevice(InputDevice inputDevice) {
        this.inputDevice = inputDevice;
        configureBindings();
    }

    public void setAutonomous(int id) {
        autonId = id;
    }
}
