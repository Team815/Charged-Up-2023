// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class SwerveModule {
    private final CANSparkMax spinController;
    private final CANSparkMax rotateController;
    private final CANCoder rotateSensor;
    private final PIDController pid;
    private final double rotationOffset;

    public SwerveModule(int spinControllerID, int rotateControllerID, int rotateSensorID, double rotationOffset) {
        spinController = new CANSparkMax(spinControllerID, MotorType.kBrushless);
        rotateController = new CANSparkMax(rotateControllerID, MotorType.kBrushless);
        rotateSensor = new CANCoder(rotateSensorID);
        this.rotationOffset = rotationOffset;
        pid = new PIDController(0.01, 0, 0);
        pid.enableContinuousInput(0, 360);
    }

    public void drive(double spinSpeed, double rotation) {
        spinController.set(spinSpeed);
        pid.setSetpoint(rotation);
        double response = -pid.calculate(rotateSensor.getAbsolutePosition());
        rotateController.set(Math.min(0.2, Math.abs(response)) * Math.signum(response));
    }

    public SwerveModuleState optimize(SwerveModuleState state) {
        return SwerveModuleState.optimize(
            new SwerveModuleState(
                state.speedMetersPerSecond,
                Rotation2d.fromDegrees(state.angle.getDegrees() + rotationOffset)),
            Rotation2d.fromDegrees(rotateSensor.getAbsolutePosition()));
    }


    public void print(){
        System.out.println("absolute position; "+rotateSensor.getAbsolutePosition()+ ", position; "+rotateSensor.getPosition());
    }
}
