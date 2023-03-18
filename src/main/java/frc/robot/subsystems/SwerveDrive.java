// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.MathUtil;
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
import frc.robot.commands.LevelChargeStation;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.stream.IntStream;

public class SwerveDrive extends SubsystemBase {
    private static boolean autoCorrectEnabled;
    private static double autoCorrectDelay;
    private static double maxLinearAcceleration;
    private static double maxAngularAcceleration;
    private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
    private static final GenericEntry autoCorrectEnabledEntry;
    private static final GenericEntry autoCorrectDelayEntry;
    private static final GenericEntry maxLinearAccelerationEntry;
    private static final GenericEntry maxAngularAccelerationEntry;
    private final Pigeon2 gyro;
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private Pose2d pose;
    private final PIDController pidRotation;
    private final Timer timer;

    static {
        autoCorrectEnabled = true;
        autoCorrectDelay = 0.4d;
        maxLinearAcceleration = 0.03d;
        maxAngularAcceleration = 0.05d;
        var tab = Shuffleboard.getTab("SmartDashboard");
        var layout = tab
            .getLayout("Swerve Drive", BuiltInLayouts.kGrid)
            .withSize(2, 2)
            .withProperties(Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 4));
        autoCorrectEnabledEntry = layout
            .add("Auto Correct", autoCorrectEnabled)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(0, 0)
            .getEntry();
        autoCorrectDelayEntry = layout
            .add("Auto Correct Delay", autoCorrectDelay)
            .withPosition(0, 1)
            .getEntry();
        maxLinearAccelerationEntry = layout
            .add("Max Linear Acceleration", maxLinearAcceleration)
            .withPosition(0, 2)
            .getEntry();
        maxAngularAccelerationEntry = layout
            .add("Max Angular Acceleration", maxAngularAcceleration)
            .withPosition(0, 3)
            .getEntry();
    }

    /**
     * Creates a new SwerveDrive.
     */
    public SwerveDrive(
        SwerveModule moduleFrontLeft,
        SwerveModule moduleFrontRight,
        SwerveModule moduleBackLeft,
        SwerveModule moduleBackRight) {
        pidRotation = new PIDController(0.01d, 0d, 0d);
        pidRotation.enableContinuousInput(0d, 360d);
        gyro = new Pigeon2(0);
        resetGyro(0d);
        modules = new SwerveModule[]{
            moduleFrontLeft,
            moduleFrontRight,
            moduleBackLeft,
            moduleBackRight
        };
        final var halfLength = 0.368d;
        final var halfWidth = 0.368d;
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
            autoCorrectDelayEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> autoCorrectDelay = e.valueData.value.getDouble());
        inst.addListener(
            maxLinearAccelerationEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> maxLinearAcceleration = e.valueData.value.getDouble());
        inst.addListener(
            maxAngularAccelerationEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> maxAngularAcceleration = e.valueData.value.getDouble());
    }

    @Override
    public void periodic() {
        pose = odometry.update(
            Rotation2d.fromDegrees(gyro.getYaw()),
            getSwerveModulePositions(modules));
    }

    public void drive(double xSpeed, double ySpeed, double angularSpeed, double maxCorrectionSpeed) {
        var yaw = gyro.getYaw();

        var targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(xSpeed, ySpeed, angularSpeed),
            Rotation2d.fromDegrees(yaw));

        var nextAngularSpeed = getNextSpeed(
            currentSpeeds.omegaRadiansPerSecond,
            targetSpeeds.omegaRadiansPerSecond,
            maxAngularAcceleration);

        var autoCorrect = autoCorrectEnabled ? autoCorrectRotation(
                currentSpeeds.omegaRadiansPerSecond,
                nextAngularSpeed,
                yaw,
                maxCorrectionSpeed) : 0d;

        currentSpeeds = new ChassisSpeeds(
            getNextSpeed(currentSpeeds.vxMetersPerSecond, targetSpeeds.vxMetersPerSecond, maxLinearAcceleration),
            getNextSpeed(currentSpeeds.vyMetersPerSecond, targetSpeeds.vyMetersPerSecond, maxLinearAcceleration),
            nextAngularSpeed);

        var autoCorrectSpeeds = new ChassisSpeeds(
            currentSpeeds.vxMetersPerSecond,
            currentSpeeds.vyMetersPerSecond,
            currentSpeeds.omegaRadiansPerSecond + autoCorrect);

        var states = kinematics.toSwerveModuleStates(autoCorrectSpeeds);

        IntStream
            .range(0, modules.length)
            .mapToObj(i -> new Pair<>(modules[i], states[i]))
            .parallel()
            .forEach(pair -> pair.getFirst().drive(pair.getSecond()));
    }

    public void resetGyro(double offset) {
        gyro.setYaw(0d + offset);
        pidRotation.setSetpoint(0d);
    }

    public void resetPose() {
        odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()),
            getSwerveModulePositions(modules),
            new Pose2d(0d, 0d, Rotation2d.fromDegrees(gyro.getYaw())));
    }

    public Pose2d getPose() {
        return pose;
    }

    public CommandBase driveOntoChargeStation() {
        return new InstantCommand(this::resetPose)
            .andThen(new DriveToCommand(
                new Pose2d(40d, 0d, Rotation2d.fromDegrees(0d)),
                0.15d,
                0.5d,
                this))
            .andThen(new LevelChargeStation(this));
    }

    public CommandBase chargeStation() {
        return new InstantCommand(() -> resetGyro(180))
            .andThen(this::resetPose)
            .andThen(new DriveToCommand(
                new Pose2d(-40d, 0d, Rotation2d.fromDegrees(180d)),
                0.2d,
                0.2d,
                this))
            .andThen(new LevelChargeStation(this));
    }

    public DriveToCommand driveTo(
        double x,
        double y,
        double angle,
        double maxLinearSpeed,
        double maxAngularSpeed) {
        return new DriveToCommand(
            new Pose2d(x, y, Rotation2d.fromDegrees(angle)),
            maxLinearSpeed,
            maxAngularSpeed,
            this
        );
    }

    public void setAngle(double angle) {
        pidRotation.setSetpoint(angle);
    }

    public double getLevel() {
        return gyro.getRoll();
    }

    public CommandBase myCommand() {
        var config = new TrajectoryConfig(0.5d, 1d).setKinematics(kinematics);

        var exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0d, 0d, Rotation2d.fromDegrees(0d)),
                List.of(new Translation2d(1d, 1d), new Translation2d(2d, -1d), new Translation2d(1d, 1d)),
                new Pose2d(0d, 0d, Rotation2d.fromDegrees(0d)),
                config);
        return new SwerveControllerCommand(
            exampleTrajectory,
            () -> pose,
            kinematics,
            new PIDController(0.1d, 0d, 0d),
            new PIDController(0.1d, 0d, 0d),
            new ProfiledPIDController(0.1d, 0d, 0d, new TrapezoidProfile.Constraints(1d, 1000d)),
            (st) -> {
                for (var i = 0; i < modules.length; i++) {
                    modules[i].drive(st[i]);
                }
            },
            this);
    }

    private double autoCorrectRotation(
        double currentAngularSpeed,
        double nextAngularSpeed,
        double yaw,
        double limit) {
        var stoppedRotating = nextAngularSpeed == 0d && currentAngularSpeed != 0d;
        if (stoppedRotating) {
            timer.start(autoCorrectDelay);
        } else if (currentAngularSpeed != 0d || timer.isRunning()) {
            setAngle(yaw);
        } else {
            return MathUtil.clamp(-pidRotation.calculate(yaw), -limit, limit);
        }
        return 0d;
    }

    private static SwerveModulePosition[] getSwerveModulePositions(SwerveModule[] modules) {
        return Arrays
            .stream(modules)
            .map(SwerveModule::getPosition)
            .toArray(SwerveModulePosition[]::new);
    }

    private static double getNextSpeed(double currentSpeed, double targetSpeed, double maxAcceleration) {
        var difference = targetSpeed - currentSpeed;
        return currentSpeed + Math.min(maxAcceleration, Math.abs(difference)) * Math.signum(difference);
    }
}
