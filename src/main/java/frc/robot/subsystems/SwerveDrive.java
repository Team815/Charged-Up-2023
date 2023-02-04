// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.ArrayList;
import java.util.List;

public class SwerveDrive extends SubsystemBase {
    private final Pigeon2 gyro;
    private final SwerveModule moduleFrontLeft;
    private final SwerveModule moduleFrontRight;
    private final SwerveModule moduleBackLeft;
    private final SwerveModule moduleBackRight;
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private Pose2d pose;

    private final ProfiledPIDController pidRotation1;
    private final PIDController pidRotation;
    private final PIDController pidX;
    private final PIDController pidY;

    /**
     * Creates a new SwerveDrive.
     */
    public SwerveDrive(
        SwerveModule moduleFrontLeft,
        SwerveModule moduleFrontRight,
        SwerveModule moduleBackLeft,
        SwerveModule moduleBackRight) {
        pidRotation1 = new ProfiledPIDController(
            0.01,
            0,
            0,
            new TrapezoidProfile.Constraints(100, 100));
        pidRotation = new PIDController(0.01, 0, 0);
        pidX = new PIDController(0.01, 0, 0);
        pidY = new PIDController(0.01, 0, 0);
        gyro = new Pigeon2(0);
        resetGyro();
        this.moduleFrontLeft = moduleFrontLeft;
        this.moduleFrontRight = moduleFrontRight;
        this.moduleBackLeft = moduleBackLeft;
        this.moduleBackRight = moduleBackRight;
        modules = new SwerveModule[]{
            this.moduleFrontLeft,
            this.moduleFrontRight,
            this.moduleBackLeft,
            this.moduleBackRight};
        final double x = 0.368;
        final double y = 0.368;
        kinematics = new SwerveDriveKinematics(
            new Translation2d(-x, -y),
            new Translation2d(-x, y),
            new Translation2d(x, -y),
            new Translation2d(x, y));
        odometry = new SwerveDriveOdometry(
            kinematics,
            new Rotation2d(gyro.getYaw()),
            new SwerveModulePosition[]{
                moduleFrontLeft.getPosition(),
                moduleFrontRight.getPosition(),
                moduleBackLeft.getPosition(),
                moduleBackRight.getPosition(),
            });
    }

    @Override
    public void periodic() {
        pose = odometry.update(
            Rotation2d.fromDegrees(gyro.getYaw()),
            new SwerveModulePosition[]{
                moduleFrontLeft.getPosition(),
                moduleFrontRight.getPosition(),
                moduleBackLeft.getPosition(),
                moduleBackRight.getPosition(),
            });
        // System.out.println(pose.getX());
        System.out.println(pose.getY());
    }

    public void drive(double speedX, double speedY, double rotation) {
        speedX = cleanInput(speedX);
        speedY = cleanInput(speedY);
        rotation = cleanInput(rotation);
        double yaw = gyro.getYaw();

        if (rotation != 0) {
            pidRotation.setSetpoint(yaw);
        } else {
            rotation -= pidRotation.calculate(yaw);
        }

        ChassisSpeeds newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(speedY, speedX, rotation),
            Rotation2d.fromDegrees(yaw));

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(newSpeeds);

        for (int i = 0; i < 4; i++) {
            modules[i].drive(states[i]);
        }
    }

    public void resetGyro() {
        gyro.setYaw(0);
        pidRotation.setSetpoint(0);
    }

    public CommandBase myCommand() {
        List<Trajectory.State> states = new ArrayList<>();
        states.add(new Trajectory.State(
            2.0,
            .5,
            0,
            new Pose2d(20, 0, Rotation2d.fromDegrees(30)),
            0));
        Trajectory trajectory = new Trajectory(states);

        return new SwerveControllerCommand(
            trajectory,
            () -> pose,
            kinematics,
            pidX,
            pidY,
            pidRotation1,
            () -> Rotation2d.fromDegrees(30),
            (st) -> {
                for (int i = 0; i < 4; i++) {
                    modules[i].drive(st[i]);
                }
            },
            this);
    }

    private static double cleanInput(double input) {
        final double deadzone = 0.15;
        double multiple = 1 / (1 - deadzone);
        double cleanedInput = (Math.abs(input) - deadzone) * multiple;
        return Math.max(0, cleanedInput) * Math.signum(input);
    }
}
