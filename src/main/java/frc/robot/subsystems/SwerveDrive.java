// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;

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
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

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
            new Rotation2d(gyro.getYaw()),
            new SwerveModulePosition[]{
                moduleFrontLeft.getPosition(),
                moduleFrontRight.getPosition(),
                moduleBackLeft.getPosition(),
                moduleBackRight.getPosition(),
            });
        timer = new Timer();

        this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
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
        speedX = cleanInput(speedX);
        speedY = cleanInput(speedY);
        rotation = cleanInput(rotation);

        double yaw = gyro.getYaw();

        //Autocorrect for Drifting
        //boolean stoppedRotating = rotation == 0 && previousRotation != 0;
        //previousRotation = rotation;
        //if (stoppedrotating) {
        //    timer.start(.2);
        //} else if (rotation != 0 || timer.isrunning()) {
        //    pidrotation.setsetpoint(yaw);
        //} else {
        //    rotation -= pidrotation.calculate(yaw);
        //}

        // 3. Make the driving smoother
        speedX = xLimiter.calculate(speedX) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        speedY = yLimiter.calculate(speedY) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        rotation = turningLimiter.calculate(rotation) * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;


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
        TrajectoryConfig config = new TrajectoryConfig(.5, 1).setKinematics(kinematics);

        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(/*new Translation2d(1, 1), new Translation2d(2, -1)*/),
                new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
                config);
        System.out.println(exampleTrajectory.getStates().size());

        return new SwerveControllerCommand(
            exampleTrajectory,
            () -> pose,
            kinematics,
            new PIDController(0.01, 0, 0),
            new PIDController(0.01, 0, 0),
            new ProfiledPIDController(0.01, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI)),
           // () -> Rotation2d.fromDegrees(30),
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
