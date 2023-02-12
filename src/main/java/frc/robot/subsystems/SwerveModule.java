// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.function.Consumer;

/**
 * Add your docs here.
 */
public class SwerveModule {
    private static final GenericEntry maxLinearRateEntry;
    private static final GenericEntry maxAngularRateEntry;
    private static final GenericEntry angularPEntry;
    private static final GenericEntry maxLinearSpeedEntry;
    private static final GenericEntry maxAngularSpeedEntry;
    private final CANSparkMax spinController;
    private final CANSparkMax rotateController;
    private final CANCoder rotateSensor;
    private final PIDController pid;

    static {
        final double maxLinearRate = 0.1;
        final double maxAngularRate = 0.1;
        final double angularP = 0.015;
        final double maxLinearSpeed = 1;
        final double maxAngularSpeed = 0.2;
        var tab = Shuffleboard.getTab("SmartDashboard");
        var layout = tab.getLayout("Swerve Modules", BuiltInLayouts.kList).withSize(2, 3);
        maxLinearRateEntry = layout.add("Max Linear Rate", maxLinearRate).getEntry();
        maxAngularRateEntry = layout.add("Max Angular Rate", maxAngularRate).getEntry();
        angularPEntry = layout.add("Angular P", angularP).getEntry();
        maxLinearSpeedEntry = layout.add("Max Linear Speed", maxLinearSpeed).getEntry();
        maxAngularSpeedEntry = layout.add("Max Angular Speed", maxAngularSpeed).getEntry();
    }

    public SwerveModule(
        int spinControllerID, int rotateControllerID, int rotateSensorID, double angularOffset) {
        spinController = new CANSparkMax(spinControllerID, MotorType.kBrushless);
        rotateController = new CANSparkMax(rotateControllerID, MotorType.kBrushless);
        spinController.restoreFactoryDefaults();
        rotateController.restoreFactoryDefaults();
        rotateSensor = new CANCoder(rotateSensorID);
        rotateSensor.configFactoryDefault();
        rotateSensor.configMagnetOffset(angularOffset);
        pid = new PIDController(angularPEntry.get().getDouble(), 0, 0);
        pid.enableContinuousInput(0, 360);
    }

    public void drive(SwerveModuleState state) {
        update(maxLinearRateEntry, spinController::setOpenLoopRampRate);
        update(maxAngularRateEntry, rotateController::setOpenLoopRampRate);
        update(angularPEntry, pid::setP);
        state = optimize(state);
        double spinSpeed = state.speedMetersPerSecond * maxLinearSpeedEntry.get().getDouble();
        double rotation = state.angle.getDegrees();
        spinController.set(spinSpeed);

        pid.setSetpoint(rotation);
        double response = -pid.calculate(rotateSensor.getAbsolutePosition());
        double maxAngularSpeed = 1 * maxAngularSpeedEntry.get().getDouble();
        rotateController.set(MathUtil.clamp(response, -maxAngularSpeed, maxAngularSpeed));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            spinController.getEncoder().getPosition(),
            Rotation2d.fromDegrees(rotateSensor.getAbsolutePosition()));
    }

    private SwerveModuleState optimize(SwerveModuleState state) {
        return SwerveModuleState.optimize(
            new SwerveModuleState(
                state.speedMetersPerSecond,
                Rotation2d.fromDegrees(state.angle.getDegrees())),
            Rotation2d.fromDegrees(rotateSensor.getAbsolutePosition()));
    }

    private void update(GenericEntry entry, Consumer<Double> consumer) {
        var values = entry.readQueue();
        if (values.length != 0) {
            consumer.accept(values[0].getDouble());
        }
    }
}
