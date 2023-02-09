// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class SwerveModule {
    private final CANSparkMax spinController;
    private final CANSparkMax rotateController;
    private final CANCoder rotateSensor;
    private final PIDController pid;

    public SwerveModule(int spinControllerID, int rotateControllerID, int rotateSensorID) {
        spinController = new CANSparkMax(spinControllerID, MotorType.kBrushless);
        rotateController = new CANSparkMax(rotateControllerID, MotorType.kBrushless);
        rotateSensor = new CANCoder(rotateSensorID);
        pid = new PIDController(0.015, 0, 0);
        pid.enableContinuousInput(0, 360);
    }

    public void drive(SwerveModuleState state) {
        state = optimize(state);
        double spinSpeed = state.speedMetersPerSecond;
        double rotation = state.angle.getDegrees();
        spinController.set(spinSpeed * Constants.DriveConstants.kPhysicalMaxSpeedPercent);

        pid.setSetpoint(rotation);
        double response = -pid.calculate(rotateSensor.getAbsolutePosition());
        rotateController.set(Math.min(0.2, Math.abs(response)) * Math.signum(response));
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
