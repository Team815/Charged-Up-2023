// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;
import java.util.List;

public final class Autos {
    /**
     * Example static factory for an autonomous command.
     */

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static CommandBase driveOverChargeStation(SwerveDrive swerveDrive) {
        var driveCom = new SwerveDriveCommander(swerveDrive);
        return new InstantCommand(swerveDrive::resetPose)
            .andThen(
                new InstantCommand(() -> swerveDrive.resetGyro(180)),
                driveCom.driveTo(0 ,0, 90, 0.2, 0.2),
                driveCom.driveTo(90, 0, 90, .4, .4),
                driveCom.driveTo(50, 0, 90, .3, .3),
                driveCom.level());
    }
}
