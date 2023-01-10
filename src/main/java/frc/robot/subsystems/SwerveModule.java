// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class SwerveModule {
    private CANSparkMax spinController;
    private CANSparkMax rotateController;

    public SwerveModule(int spinControllerID, int rotateControllerID) {
        spinController = new CANSparkMax(spinControllerID, MotorType.kBrushless);
        rotateController = new CANSparkMax(rotateControllerID, MotorType.kBrushless);
    }
}
