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
        final double maxLinearRate = 0.1;
        final double maxAngularRate = 0.1;
        final double angularP = 0.015;
        maxLinearSpeed = 1;
        maxAngularSpeed = 0.2;
        var tab = Shuffleboard.getTab("SmartDashboard");
        var layout = tab.getLayout("Swerve Modules", BuiltInLayouts.kList).withSize(2, 3);
        maxLinearSpeedEntry = layout.add("Max Linear Speed", maxLinearSpeed).getEntry();
        maxAngularSpeedEntry = layout.add("Max Angular Speed", maxAngularSpeed).getEntry();
        maxLinearRateEntry = layout.add("Max Linear Rate", maxLinearRate).getEntry();
        maxAngularRateEntry = layout.add("Max Angular Rate", maxAngularRate).getEntry();
        angularPEntry = layout.add("Angular P", angularP).getEntry();
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
        double spinSpeed = state.speedMetersPerSecond * maxLinearSpeed;
        double rotation = state.angle.getDegrees();
        spinController.set(spinSpeed);

        pid.setSetpoint(rotation);
        double response = -pid.calculate(rotateSensor.getAbsolutePosition());
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
