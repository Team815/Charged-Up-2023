package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class AutoCorrectDrive extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final DoubleSupplier forwardVelocitySupplier;
    private final DoubleSupplier sidewaysVelocitySupplier;
    private final DoubleSupplier angularVelocitySupplier;
    private final PIDController pid = new PIDController(0.01d, 0d, 0d);

    public AutoCorrectDrive(
        SwerveDrive swerveDrive,
        DoubleSupplier forwardVelocitySupplier,
        DoubleSupplier sidewaysVelocitySupplier,
        DoubleSupplier angularVelocitySupplier) {
        this.swerveDrive = swerveDrive;
        this.forwardVelocitySupplier = forwardVelocitySupplier;
        this.sidewaysVelocitySupplier = sidewaysVelocitySupplier;
        this.angularVelocitySupplier = angularVelocitySupplier;
        pid.enableContinuousInput(0d, 360d);
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        var angularVelocity = angularVelocitySupplier.getAsDouble();
        var yaw = swerveDrive.getAngles().getYaw();
        if (angularVelocity != 0) {
            pid.setSetpoint(yaw);
        } else {
            angularVelocity = pid.calculate(yaw);
        }
        swerveDrive.drive(
            forwardVelocitySupplier.getAsDouble(),
            sidewaysVelocitySupplier.getAsDouble(),
            angularVelocity);
    }
}
