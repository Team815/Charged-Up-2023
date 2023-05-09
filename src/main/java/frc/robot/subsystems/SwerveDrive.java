// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;

public class SwerveDrive extends SubsystemBase {
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private final Pigeon2 gyro;
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    /**
     * Creates a new SwerveDrive.
     */
    public SwerveDrive(Pigeon2 gyro, SwerveDriveModule... modules) {
        this.gyro = gyro;
        resetGyro(0d);
        this.modules = Arrays
            .stream(modules)
            .map(SwerveDriveModule::module)
            .toArray(SwerveModule[]::new);
        kinematics = new SwerveDriveKinematics(Arrays
            .stream(modules)
            .map(SwerveDriveModule::translation)
            .toArray(Translation2d[]::new));
        odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(gyro.getYaw()),
            getSwerveModulePositions(this.modules));
    }

    @Override
    public void periodic() {
        odometry.update(
            Rotation2d.fromDegrees(gyro.getYaw()),
            getSwerveModulePositions(modules));
    }

    public void drive(double forwardVelocity, double sidewaysVelocity, double angularVelocity) {
        speeds = new ChassisSpeeds(forwardVelocity, sidewaysVelocity, angularVelocity);
        var robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(gyro.getYaw()));
        var states = kinematics.toSwerveModuleStates(robotSpeeds);

        for (var i = 0; i < modules.length; i++) {
            modules[i].drive(states[i]);
        }
    }

    public void resetGyro(double offset) {
        gyro.setYaw(0d + offset);
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
        return speeds;
    }

    public GyroAngles getAngles() {
        return new GyroAngles(gyro.getPitch(), gyro.getRoll(), gyro.getYaw());
    }

    private static SwerveModulePosition[] getSwerveModulePositions(SwerveModule[] modules) {
        return Arrays
            .stream(modules)
            .map(SwerveModule::getPosition)
            .toArray(SwerveModulePosition[]::new);
    }
}
