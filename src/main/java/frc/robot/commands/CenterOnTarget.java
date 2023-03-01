package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class CenterOnTarget extends CommandBase {

    SwerveDrive swerveDrive;
    DoubleSupplier targetOffsetSupplier;
    DoubleSupplier speedXSupplier;
    DoubleSupplier speedYSupplier;
    PIDController pid;

    public CenterOnTarget(
            SwerveDrive swerveDrive,
            DoubleSupplier targetOffsetSupplier,
            DoubleSupplier speedXSupplier,
            DoubleSupplier speedYSupplier) {
        this.swerveDrive = swerveDrive;
        this.targetOffsetSupplier = targetOffsetSupplier;
        this.speedXSupplier = speedXSupplier;
        this.speedYSupplier = speedYSupplier;
        pid = new PIDController(0.01d, 0d, 0d);
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        var rotation = pid.calculate(targetOffsetSupplier.getAsDouble());
        swerveDrive.drive(speedXSupplier.getAsDouble(), speedYSupplier.getAsDouble(), rotation, 0.5);
    }
}
