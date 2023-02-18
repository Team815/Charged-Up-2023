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
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.EnumSet;
import java.util.Map;

/**
 * Add your docs here.
 */
public class SwerveModule {
    private static final GenericEntry maxLinearRateEntry;
    private static final GenericEntry maxAngularRateEntry;
    private static final GenericEntry angularPEntry;
    private static final GenericEntry maxLinearSpeedEntry;
    private static final GenericEntry maxAngularSpeedEntry;
    private static double maxLinearSpeed;
    private static double maxAngularSpeed;
    private final CANSparkMax spinController;
    private final CANSparkMax rotateController;
    private final CANCoder rotateSensor;
    private final PIDController pid;

    static {
        final var maxLinearRate = 0.1d;
        final var maxAngularRate = 0.1d;
        final var angularP = 0.015d;
        maxLinearSpeed = 1d;
        maxAngularSpeed = 0.2d;
        var tab = Shuffleboard.getTab("SmartDashboard");
        var layout = tab
            .getLayout("Swerve Modules", BuiltInLayouts.kGrid)
            .withSize(2, 2)
            .withProperties(Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 5));
        maxLinearSpeedEntry = layout
            .add("Max Linear Speed", maxLinearSpeed)
            .withPosition(0, 0)
            .getEntry();
        maxAngularSpeedEntry = layout
            .add("Max Angular Speed", maxAngularSpeed)
            .withPosition(0, 1)
            .getEntry();
        maxLinearRateEntry = layout
            .add("Max Linear Rate", maxLinearRate)
            .withPosition(0, 2)
            .getEntry();
        maxAngularRateEntry = layout
            .add("Max Angular Rate", maxAngularRate)
            .withPosition(0, 3)
            .getEntry();
        angularPEntry = layout
            .add("Angular P", angularP)
            .withPosition(0, 4)
            .getEntry();
    }

    public SwerveModule(int spinControllerID, int rotateControllerID, int rotateSensorID, double angularOffset) {
        spinController = new CANSparkMax(spinControllerID, MotorType.kBrushless);
        rotateController = new CANSparkMax(rotateControllerID, MotorType.kBrushless);
        spinController.restoreFactoryDefaults();
        rotateController.restoreFactoryDefaults();
        rotateSensor = new CANCoder(rotateSensorID);
        rotateSensor.configFactoryDefault();
        rotateSensor.configMagnetOffset(angularOffset);
        pid = new PIDController(angularPEntry.get().getDouble(), 0d, 0d);
        pid.enableContinuousInput(0d, 360d);

        // Shuffleboard listeners

        var inst = NetworkTableInstance.getDefault();
        inst.addListener(
            maxLinearRateEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> spinController.setOpenLoopRampRate(e.valueData.value.getDouble()));
        inst.addListener(
            maxAngularRateEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> rotateController.setOpenLoopRampRate(e.valueData.value.getDouble()));
        inst.addListener(
            angularPEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> pid.setP(e.valueData.value.getDouble()));
        inst.addListener(
            maxLinearSpeedEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> maxLinearSpeed = e.valueData.value.getDouble());
        inst.addListener(
            maxAngularSpeedEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> maxAngularSpeed = e.valueData.value.getDouble());
    }

    public void drive(SwerveModuleState state) {
        state = optimize(state);
        var spinSpeed = state.speedMetersPerSecond * maxLinearSpeed;
        spinController.set(spinSpeed);

        var rotation = state.angle.getDegrees();
        pid.setSetpoint(rotation);
        var response = -pid.calculate(rotateSensor.getAbsolutePosition());
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
}
