package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class RobotCommander {
    private final SwerveDrive swerveDrive;

    public RobotCommander(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    // Swerve Drive Commands

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
}
