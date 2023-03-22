// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.stream.IntStream;

public class SwerveDrive extends SubsystemBase {
    public static final boolean DEFAULT_AUTO_CORRECT_ENABLED = true;
    public static final double DEFAULT_AUTO_CORRECT_DELAY = 0.4d;
    public static final double DEFAULT_MAX_LINEAR_ACCELERATION = 0.03d;
    public static final double DEFAULT_MAX_ANGULAR_ACCELERATION = 0.03d;
    public static final double DEFAULT_MAX_AUTO_CORRECT_SPEED = 0.5d;
    private boolean autoCorrectEnabled = DEFAULT_AUTO_CORRECT_ENABLED;
    private double autoCorrectDelay = DEFAULT_AUTO_CORRECT_DELAY;
    private double maxLinearAcceleration = DEFAULT_MAX_LINEAR_ACCELERATION;
    private double maxAngularAcceleration = DEFAULT_MAX_ANGULAR_ACCELERATION;
    private double maxAutoCorrectSpeed = DEFAULT_MAX_AUTO_CORRECT_SPEED;
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

    public void drive(double forwardVelocity, double sidewaysVelocity, double angularVelocity) {
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
            yaw) : 0d;

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

    public ChassisSpeeds getSpeeds() {
        return currentSpeeds;
    }

    public void setAngle(double angle) {
        pidRotation.setSetpoint(angle);
    }

    public GyroAngles getAngles() {
        return new GyroAngles(gyro.getPitch(), gyro.getRoll(), gyro.getYaw());
    }

    public void setAutoCorrectEnabled(boolean autoCorrectEnabled) {
        this.autoCorrectEnabled = autoCorrectEnabled;
        if (autoCorrectEnabled) {
            pidRotation.setSetpoint(gyro.getYaw());
        }
    }

    public void setAutoCorrectDelay(double autoCorrectDelay) {
        this.autoCorrectDelay = Math.abs(autoCorrectDelay);
    }

    public void setMaxLinearAcceleration(double maxLinearAcceleration) {
        this.maxLinearAcceleration = Math.abs(maxLinearAcceleration);
    }

    public void setMaxAngularAcceleration(double maxAngularAcceleration) {
        this.maxAngularAcceleration = Math.abs(maxAngularAcceleration);
    }

    public void setMaxAutoCorrectSpeed(double maxAutoCorrectSpeed) {
        this.maxAutoCorrectSpeed = MathUtil.clamp(Math.abs(maxAutoCorrectSpeed), 0, 1);
    }

    private double autoCorrectRotation(double currentAngularSpeed, double nextAngularSpeed, double yaw) {
        var stoppedRotating = nextAngularSpeed == 0d && currentAngularSpeed != 0d;
        if (stoppedRotating) {
            timer.start(autoCorrectDelay);
        } else if (currentAngularSpeed != 0d || timer.isRunning()) {
            setAngle(yaw);
        } else {
            return MathUtil.clamp(pidRotation.calculate(yaw), -maxAutoCorrectSpeed, maxAutoCorrectSpeed);
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
