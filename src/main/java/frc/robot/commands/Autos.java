// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
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

    public static CommandBase auton1(SwerveDrive swerveDrive, Shoulder shoulder, Arm arm, Claw claw) {
        return new InstantCommand(swerveDrive::resetPose)
            .andThen(new InstantCommand(() -> swerveDrive.resetGyro(180d)))
            .andThen(swerveDrive.driveTo(4d, 0d, 180d, 0.2d, 0.2d)
                .alongWith(new LiftArmTo(arm, 10d, 0.1d)))
            .andThen((new MoveShoulder(shoulder, 18000d)
                .alongWith(swerveDrive.driveTo(0d, 0d, 180d, 0.2d, 0.2d)))
                .deadlineWith(new KeepArmAt(arm, 10d, 0.2d)))
            .andThen(new WaitCommand(0.3d)
                .deadlineWith(new InstantCommand(claw::open), new KeepArmAt(arm, 10d, 0.2d)))
            .andThen(((new MoveShoulder(shoulder, 50d)
                .deadlineWith(new KeepArmAt(arm, 10d, 0.2d), new InstantCommand(claw::close)))
                .andThen(new DropArm(arm)
                    .alongWith(new PrintCommand("Dropping Time"))))
                .alongWith(swerveDrive.driveTo(85d, 0d, 180d, 0.5d, 0.5d)))
            .andThen(swerveDrive.driveTo(85d, 40d, 180d, 0.5d, 0.5d))
            .andThen(swerveDrive.driveTo(42d, 40d, 180d, 0.2d, 0.5d))
            .andThen(new LevelChargeStation(swerveDrive));
    }
}
