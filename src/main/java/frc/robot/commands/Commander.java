package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RangePidController;
import frc.robot.commands.drive.AccelerationDrive;
import frc.robot.commands.drive.AutoCorrectDrive;
import frc.robot.commands.drive.DriveTo;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class Commander {
    private final SwerveDrive swerveDrive;

    public Commander(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    public InstantCommand resetPose() {
        return new InstantCommand(swerveDrive::resetPose, swerveDrive);
    }

    public InstantCommand resetGyro(double offset) {
        return new InstantCommand(() -> swerveDrive.resetGyro(offset), swerveDrive);
    }

    public RunCommand simpleDrive(
        DoubleSupplier forwardVelocitySupplier,
        DoubleSupplier sidewaysVelocitySupplier,
        DoubleSupplier angularVelocitySupplier) {
        return new RunCommand(() -> swerveDrive.drive(
            forwardVelocitySupplier.getAsDouble(),
            sidewaysVelocitySupplier.getAsDouble(),
            angularVelocitySupplier.getAsDouble()),
            swerveDrive);
    }

    public AccelerationDrive accelerationDrive(
        DoubleSupplier forwardVelocitySupplier,
        DoubleSupplier sidewaysVelocitySupplier,
        DoubleSupplier angularVelocitySupplier) {
        return new AccelerationDrive(
            swerveDrive,
            forwardVelocitySupplier,
            sidewaysVelocitySupplier,
            angularVelocitySupplier);
    }

    public AutoCorrectDrive autoCorrectDrive(
        DoubleSupplier forwardVelocitySupplier,
        DoubleSupplier sidewaysVelocitySupplier,
        DoubleSupplier angularVelocitySupplier) {
        return new AutoCorrectDrive(
            swerveDrive,
            forwardVelocitySupplier,
            sidewaysVelocitySupplier,
            angularVelocitySupplier);
    }

    public DriveTo driveTo(double forwardPosition, double sidewaysPosition, double angularPosition) {
        var linearPid = new RangePidController(0.03d, 0d, 0d, -0.2d, 0.2d);
        var angularPid = new RangePidController(0.02d, 0d, 0d, -0.3d, 0.3d);
        linearPid.setTolerance(1d);
        angularPid.setTolerance(5d);
        angularPid.enableContinuousInput(0, 360d);
        return new DriveTo(
            swerveDrive,
            new Pose2d(forwardPosition, sidewaysPosition, Rotation2d.fromDegrees(angularPosition)),
            linearPid,
            angularPid);
    }
}
