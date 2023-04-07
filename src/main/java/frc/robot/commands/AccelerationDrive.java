package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class AccelerationDrive extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final DoubleSupplier forwardVelocitySupplier;
    private final DoubleSupplier sidewaysVelocitySupplier;
    private final DoubleSupplier angularVelocitySupplier;
    private double maxForwardAcceleration = 0.01d;
    private double maxSidewaysAcceleration = 0.01d;
    private double maxAngularAcceleration = 0.01d;

    public AccelerationDrive(
        SwerveDrive swerveDrive,
        DoubleSupplier forwardVelocitySupplier,
        DoubleSupplier sidewaysVelocitySupplier,
        DoubleSupplier angularVelocitySupplier) {
        this.swerveDrive = swerveDrive;
        this.forwardVelocitySupplier = forwardVelocitySupplier;
        this.sidewaysVelocitySupplier = sidewaysVelocitySupplier;
        this.angularVelocitySupplier = angularVelocitySupplier;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        var currentSpeeds = swerveDrive.getSpeeds();
        var forwardVelocity = getNextSpeed(
            currentSpeeds.vxMetersPerSecond,
            forwardVelocitySupplier.getAsDouble(),
            maxForwardAcceleration);
        var sidewaysVelocity = getNextSpeed(
            currentSpeeds.vyMetersPerSecond,
            sidewaysVelocitySupplier.getAsDouble(),
            maxSidewaysAcceleration);
        var angularVelocity = getNextSpeed(
            currentSpeeds.omegaRadiansPerSecond,
            angularVelocitySupplier.getAsDouble(),
            maxAngularAcceleration);
        swerveDrive.drive(forwardVelocity, sidewaysVelocity, angularVelocity);
    }

    private static double getNextSpeed(double currentSpeed, double targetSpeed, double maxAcceleration) {
        var delta = targetSpeed - currentSpeed;
        return currentSpeed + Math.min(maxAcceleration, Math.abs(delta)) * Math.signum(delta);
    }
}
