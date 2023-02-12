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
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.DriveToCommand;

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
    private final PIDController pidRotation;
    private double previousRotation;
    private final Timer timer;

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
            Rotation2d.fromDegrees(gyro.getYaw()),
            new SwerveModulePosition[]{
                moduleFrontLeft.getPosition(),
                moduleFrontRight.getPosition(),
                moduleBackLeft.getPosition(),
                moduleBackRight.getPosition(),
            });
        timer = new Timer();
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
    }

    public void drive(double speedX, double speedY, double rotation) {
        double yaw = gyro.getYaw();

        //Autocorrect for Drifting
        boolean stoppedRotating = rotation == 0 && previousRotation != 0;
        previousRotation = rotation;
        if (stoppedRotating) {
            timer.start(.2);
        } else if (rotation != 0 || timer.isRunning()) {
            pidRotation.setSetpoint(yaw);
        } else {
            rotation -= pidRotation.calculate(yaw);
        }

        ChassisSpeeds newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(speedX, speedY, rotation),
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

    public void resetPose() {
        odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()),
            new SwerveModulePosition[]{
                moduleFrontLeft.getPosition(),
                moduleFrontRight.getPosition(),
                moduleBackLeft.getPosition(),
                moduleBackRight.getPosition(),
            },
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
        TrajectoryConfig config = new TrajectoryConfig(.5, 1).setKinematics(kinematics);

        Trajectory exampleTrajectory =
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
                for (int i = 0; i < 4; i++) {
                    modules[i].drive(st[i]);
                }
            },
            this);
    }
}
