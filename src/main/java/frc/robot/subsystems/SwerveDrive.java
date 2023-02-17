// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.DriveToCommand;
import frc.robot.Limelight;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.stream.IntStream;

public class SwerveDrive extends SubsystemBase {

    private static double maxXSpeed;
    private static double maxYSpeed;
    private static double maxAngularSpeed;
    private static boolean autoCorrectEnabled;
    private static double autoCorrectDelay;
    private static final GenericEntry maxXSpeedEntry;
    private static final GenericEntry maxYSpeedEntry;
    private static final GenericEntry maxAngularSpeedEntry;
    private static final GenericEntry autoCorrectEnabledEntry;
    private static final GenericEntry autoCorrectDelayEntry;
    private final Pigeon2 gyro;
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private Pose2d pose;
    private final PIDController pidRotation;
    private double previousRotation;
    private final Timer timer;

    static {
        autoCorrectEnabled = true;
        maxXSpeed = 1;
        maxYSpeed = 1;
        maxAngularSpeed = 1;
        autoCorrectDelay = 0.2;
        var tab = Shuffleboard.getTab("SmartDashboard");
        var layout = tab.getLayout("Swerve Drive", BuiltInLayouts.kList).withSize(2, 3);
        maxXSpeedEntry = layout.add("Max X Speed", maxXSpeed).getEntry();
        maxYSpeedEntry = layout.add("Max Y Speed", maxYSpeed).getEntry();
        maxAngularSpeedEntry = layout.add("Max Angular Speed", maxAngularSpeed).getEntry();
        autoCorrectEnabledEntry = layout.add("Auto Correct", autoCorrectEnabled).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        autoCorrectDelayEntry = layout.add("Auto Correct Delay", autoCorrectDelay).getEntry();
    }

    /**
     * Creates a new SwerveDrive.
     */
    public SwerveDrive(
        SwerveModule moduleFrontLeft,
        SwerveModule moduleFrontRight,
        SwerveModule moduleBackLeft,
        SwerveModule moduleBackRight) {
        pidRotation = new PIDController(0.01, 0, 0);
        gyro = new Pigeon2(0);
        resetGyro();
        modules = new SwerveModule[]{
            moduleFrontLeft,
            moduleFrontRight,
            moduleBackLeft,
            moduleBackRight
        };
        final var halfLength = 0.368;
        final var halfWidth = 0.368;
        kinematics = new SwerveDriveKinematics(
            new Translation2d(-halfLength, -halfWidth),
            new Translation2d(-halfLength, halfWidth),
            new Translation2d(halfLength, -halfWidth),
            new Translation2d(halfLength, halfWidth));
        odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(gyro.getYaw()),
            getSwerveModulePositions(modules));
        timer = new Timer();

        // Shuffleboard listeners

        var inst = NetworkTableInstance.getDefault();
        inst.addListener(
            autoCorrectEnabledEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> {
                autoCorrectEnabled = e.valueData.value.getBoolean();
                if (autoCorrectEnabled) {
                    pidRotation.setSetpoint(gyro.getYaw());
                }
            });
        inst.addListener(
            maxXSpeedEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> maxXSpeed = e.valueData.value.getDouble());
        inst.addListener(
            maxYSpeedEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> maxYSpeed = e.valueData.value.getDouble());
        inst.addListener(
            maxAngularSpeedEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> maxAngularSpeed = e.valueData.value.getDouble());
        inst.addListener(
            autoCorrectDelayEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> autoCorrectDelay = e.valueData.value.getDouble());
    }

    @Override
    public void periodic() {
        pose = odometry.update(
            Rotation2d.fromDegrees(gyro.getYaw()),
            getSwerveModulePositions(modules));

        System.out.println("Limelight X: " + Limelight.limelightField.getX());
        System.out.println("Limelight Y: " + Limelight.limelightField.getY());
    }

    public void drive(double speedX, double speedY, double rotation) {
        var yaw = gyro.getYaw();

        if (autoCorrectEnabled) {
            rotation = autoCorrectRotation(rotation, yaw);
        }

        var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                speedX * maxXSpeed,
                speedY * maxYSpeed,
                rotation * maxAngularSpeed),
            Rotation2d.fromDegrees(yaw));

        var states = kinematics.toSwerveModuleStates(speeds);

        IntStream
            .range(0, modules.length)
            .mapToObj(i -> new Pair<>(modules[i], states[i]))
            .parallel()
            .forEach(pair -> pair.getFirst().drive(pair.getSecond()));
    }

    public void resetGyro() {
        gyro.setYaw(0);
        pidRotation.setSetpoint(0);
    }

    public void resetPose() {
        odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()),
            getSwerveModulePositions(modules),
            new Pose2d(0, 0, Rotation2d.fromDegrees(gyro.getYaw())));
    }

    public Pose2d getPose() {
        return pose;
    }

    public CommandBase driveHeart() {
        return new InstantCommand(this::resetPose)
            .andThen(new DriveToCommand(new Pose2d(20, 20, new Rotation2d()), this))
            .andThen(new DriveToCommand(new Pose2d(30, 20, new Rotation2d()), this))
            .andThen(new DriveToCommand(new Pose2d(40, 10, new Rotation2d()), this))
            .andThen(new DriveToCommand(new Pose2d(30, 0, new Rotation2d()), this))
            .andThen(new DriveToCommand(new Pose2d(40, -10, new Rotation2d()), this))
            .andThen(new DriveToCommand(new Pose2d(30, -20, new Rotation2d()), this))
            .andThen(new DriveToCommand(new Pose2d(20, -20, new Rotation2d()), this))
            .andThen(new DriveToCommand(new Pose2d(0, 0, new Rotation2d()), this));
    }

    public CommandBase myCommand() {
        var config = new TrajectoryConfig(.5, 1).setKinematics(kinematics);

        var exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(new Translation2d(1, 1), new Translation2d(2, -1), new Translation2d(1, 1)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                config);
        return new SwerveControllerCommand(
            exampleTrajectory,
            () -> pose,
            kinematics,
            new PIDController(0.1, 0, 0),
            new PIDController(0.1, 0, 0),
            new ProfiledPIDController(.1, 0, 0, new TrapezoidProfile.Constraints(1, 1000)),
            (st) -> {
                for (int i = 0; i < modules.length; i++) {
                    modules[i].drive(st[i]);
                }
            },
            this);
    }

    private double autoCorrectRotation(double rotation, double yaw) {
        var stoppedRotating = rotation == 0 && previousRotation != 0;
        previousRotation = rotation;
        if (stoppedRotating) {
            timer.start(autoCorrectDelay);
        } else if (rotation != 0 || timer.isRunning()) {
            pidRotation.setSetpoint(yaw);
        } else {
            rotation -= pidRotation.calculate(yaw);
        }
        return rotation;
    }

    private static SwerveModulePosition[] getSwerveModulePositions(SwerveModule[] modules) {
        return Arrays
            .stream(modules)
            .map(SwerveModule::getPosition)
            .toArray(SwerveModulePosition[]::new);
    }
}
