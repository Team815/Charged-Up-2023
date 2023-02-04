// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.List;

public final class Autos {
    /**
     * Example static factory for an autonomous command.
     */
    public static CommandBase exampleAuto(SwerveDrive swerveDrive) {
        return Commands.sequence(
            Commands.race(
                Commands.run(() -> swerveDrive.drive(0, 0.4, 0), swerveDrive),
                Commands.waitSeconds(2.3)),
            Commands.run(() -> swerveDrive.drive(0, 0, 0), swerveDrive));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
