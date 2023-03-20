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

public class SwerveModule {
    public static final double DEFAULT_MAX_LINEAR_SPEED = 1d;
    public static final double DEFAULT_MAX_ANGULAR_SPEED = 0.2d;
    public static final double DEFAULT_MAX_LINEAR_ACCELERATION = 0.1d;
    public static final double DEFAULT_MAX_ANGULAR_ACCELERATION = 0.1d;
    public static final double DEFAULT_P = 0.015d;
    private double maxLinearSpeed = DEFAULT_MAX_LINEAR_SPEED;
    private double maxAngularSpeed = DEFAULT_MAX_ANGULAR_SPEED;
    private final CANSparkMax linearController;
    private final CANSparkMax angularController;
    private final CANCoder rotateSensor;
    private final PIDController pid = new PIDController(DEFAULT_P, 0d, 0d);

    public SwerveModule(int spinControllerID, int rotateControllerID, int rotateSensorID, double angularOffset) {
        linearController = new CANSparkMax(spinControllerID, MotorType.kBrushless);
        angularController = new CANSparkMax(rotateControllerID, MotorType.kBrushless);
        linearController.restoreFactoryDefaults();
        angularController.restoreFactoryDefaults();
        setMaxLinearAcceleration(DEFAULT_MAX_LINEAR_ACCELERATION);
        setMaxAngularAcceleration(DEFAULT_MAX_ANGULAR_ACCELERATION);
        rotateSensor = new CANCoder(rotateSensorID);
        rotateSensor.configFactoryDefault();
        rotateSensor.configMagnetOffset(angularOffset);
        pid.enableContinuousInput(0d, 360d);
    }

    public void drive(SwerveModuleState state) {
        state = optimize(state);
        var spinSpeed = MathUtil.clamp(state.speedMetersPerSecond, -maxLinearSpeed, maxLinearSpeed);
        linearController.set(spinSpeed);

        var rotation = state.angle.getDegrees();
        pid.setSetpoint(rotation);
        var response = -pid.calculate(rotateSensor.getAbsolutePosition());
        angularController.set(MathUtil.clamp(response, -maxAngularSpeed, maxAngularSpeed));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            linearController.getEncoder().getPosition(),
            Rotation2d.fromDegrees(rotateSensor.getAbsolutePosition()));
    }

    public void setMaxLinearSpeed(double maxLinearSpeed) {
        this.maxLinearSpeed = MathUtil.clamp(Math.abs(maxLinearSpeed), 0, 1);
    }

    public void setMaxAngularSpeed(double maxAngularSpeed) {
        this.maxAngularSpeed = MathUtil.clamp(Math.abs(maxAngularSpeed), 0, 1);
    }

    public void setP(double p) {
        pid.setP(p);
    }

    public void setMaxLinearAcceleration(double maxLinearAcceleration) {
        linearController.setOpenLoopRampRate(Math.abs(maxLinearAcceleration));
    }

    public void setMaxAngularAcceleration(double maxAngularAcceleration) {
        angularController.setOpenLoopRampRate(Math.abs(maxAngularAcceleration));
    }

    private SwerveModuleState optimize(SwerveModuleState state) {
        return SwerveModuleState.optimize(
            new SwerveModuleState(
                state.speedMetersPerSecond,
                Rotation2d.fromDegrees(state.angle.getDegrees())),
            Rotation2d.fromDegrees(rotateSensor.getAbsolutePosition()));
    }
}
