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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.DriveToCommand;

import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

public class SwerveDrive extends SubsystemBase {
    public static final boolean DEFAULT_AUTO_CORRECT_ENABLED = true;
    public static final double DEFAULT_AUTO_CORRECT_DELAY = 0.4d;
    public static final double DEFAULT_MAX_LINEAR_ACCELERATION = 0.03d;
    public static final double DEFAULT_MAX_ANGULAR_ACCELERATION = 0.03d;
    private boolean autoCorrectEnabled = DEFAULT_AUTO_CORRECT_ENABLED;
    private double autoCorrectDelay = DEFAULT_AUTO_CORRECT_DELAY;
    private double maxLinearAcceleration = DEFAULT_MAX_LINEAR_ACCELERATION;
    private double maxAngularAcceleration = DEFAULT_MAX_ANGULAR_ACCELERATION;
    private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
    private final Pigeon2 gyro;
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final PIDController pidRotation;
    private final Timer timer;

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
            new Translation2d(halfLength, halfWidth),
            new Translation2d(halfLength, -halfWidth),
            new Translation2d(-halfLength, halfWidth),
            new Translation2d(-halfLength, -halfWidth));
        odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(gyro.getYaw()),
            getSwerveModulePositions(modules));
        timer = new Timer();
    }

    @Override
    public void periodic() {
        odometry.update(
            Rotation2d.fromDegrees(gyro.getYaw()),
            getSwerveModulePositions(modules));
    }

    public void drive(double forwardVelocity, double sidewaysVelocity, double angularVelocity, double maxCorrectionSpeed) {
        var yaw = gyro.getYaw();

        var targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(forwardVelocity, sidewaysVelocity, angularVelocity),
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
        setSwerveModuleStates(states);
    }

    public void resetGyro(double offset) {
        gyro.setYaw(0d + offset);
        pidRotation.setSetpoint(0d);
    }

    public void resetPose() {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.getYaw()),
            getSwerveModulePositions(modules),
            new Pose2d(0d, 0d, Rotation2d.fromDegrees(gyro.getYaw())));
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
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

    public CommandBase MySwerveControllerCommand() {
        var config = new TrajectoryConfig(2d, 0.2d).setKinematics(kinematics);

        var exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(5, 5), new Translation2d(10, -5)),
                new Pose2d(10, 0, new Rotation2d(0)),
                config);
        var thetaController =
            new ProfiledPIDController(
                0.1d, 0, 0, new TrapezoidProfile.Constraints(1d, 1d));
        thetaController.enableContinuousInput(0, 360);

        System.out.println(exampleTrajectory.getStates().size());
        for (var state : exampleTrajectory.getStates()) {
            System.out.println(state.toString());
        }

        return new SwerveControllerCommand(
            exampleTrajectory,
            this::getPose,
            kinematics,
            new PIDController(0.1d, 0d, 0d),
            new PIDController(0.1d, 0d, 0d),
            thetaController,
            this::setSwerveModuleStates,
            this);
    }

    public void setAutoCorrectEnabled(boolean autoCorrectEnabled) {
        this.autoCorrectEnabled = autoCorrectEnabled;
        if (autoCorrectEnabled) {
            pidRotation.setSetpoint(gyro.getYaw());
        }
    }

    public void setAutoCorrectDelay(double autoCorrectDelay) {
        this.autoCorrectDelay = autoCorrectDelay;
    }

    public void setMaxLinearAcceleration(double maxLinearAcceleration) {
        this.maxLinearAcceleration = maxLinearAcceleration;
    }

    public void setMaxAngularAcceleration(double maxAngularAcceleration) {
        this.maxAngularAcceleration = maxAngularAcceleration;
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
            return MathUtil.clamp(pidRotation.calculate(yaw), -limit, limit);
        }
        return 0d;
    }

    private static SwerveModulePosition[] getSwerveModulePositions(SwerveModule[] modules) {
        return Arrays
            .stream(modules)
            .map(SwerveModule::getPosition)
            .toArray(SwerveModulePosition[]::new);
    }

    private void setSwerveModuleStates (SwerveModuleState[] states) {
        IntStream
            .range(0, modules.length)
            .mapToObj(i -> new Pair<>(modules[i], states[i]))
            .parallel()
            .forEach(pair -> pair.getFirst().drive(pair.getSecond()));
    }

    private static double getNextSpeed(double currentSpeed, double targetSpeed, double maxAcceleration) {
        var difference = targetSpeed - currentSpeed;
        return currentSpeed + Math.min(maxAcceleration, Math.abs(difference)) * Math.signum(difference);
    }
}
