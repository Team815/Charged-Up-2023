// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.SwerveDrive;

public final class Autos {
    /**
     * Example static factory for an autonomous command.
     */

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static final double chargeStationPositionX = 30d;
    private static final double chargeStationPositionY = 40d;

    private static CommandBase score(SwerveDrive swerveDrive, Shoulder shoulder, Arm arm, Claw claw) {
        return new InstantCommand(swerveDrive::resetPose)
            .andThen(new InstantCommand(() -> swerveDrive.resetGyro(180d)))
            .andThen(swerveDrive.driveTo(4d, 0d, 180d, 0.2d, 0.2d)
                .alongWith(new LiftArmTo(arm, KeepArmAt.FarConeNode, 0.1d)))
            .andThen(new MoveShoulder(shoulder, 18000d)
                .alongWith(swerveDrive.driveTo(0d, 0d, 180d, 0.2d, 0.2d))
                .deadlineWith(new KeepArmAt(arm, KeepArmAt.FarConeNode, 0.2d)))
            .withTimeout(4d)
            .andThen(new WaitCommand(0.3d)
                .deadlineWith(new InstantCommand(claw::open), new KeepArmAt(arm, KeepArmAt.FarConeNode, 0.2d)));
    }

    public static CommandBase scoreCross(SwerveDrive swerveDrive, Shoulder shoulder, Arm arm, Claw claw) {
        return score(swerveDrive, shoulder, arm, claw)
            .andThen(new MoveShoulder(shoulder, 0d)
                .alongWith(
                    swerveDrive.driveTo(85d, 0d, 180d, 0.6d, 0.5d),
                    new InstantCommand(claw::close),
                    new WaitCommand(0.3d)
                        .deadlineWith(new KeepArmAt(arm, KeepArmAt.FarConeNode, 0.2d))
                        .andThen(new DropArm(arm))));
    }

    private static CommandBase scoreCrossLevel(SwerveDrive swerveDrive, Shoulder shoulder, Arm arm, Claw claw, int direction) {
        return scoreCross(swerveDrive, shoulder, arm, claw)
            .andThen(swerveDrive.driveTo(85d, chargeStationPositionY * Math.signum(direction), 90d, 0.6d, 0.5d))
            .andThen(swerveDrive.driveTo(chargeStationPositionX, chargeStationPositionY * Math.signum(direction), 90d, 0.3d, 0.5d))
            .andThen(new LevelChargeStation(swerveDrive))
            .withTimeout(14.8d)
            .andThen(new RunCommand(() -> swerveDrive.drive(0d, 0d, 0d, 0d)));
    }

    public static CommandBase scoreCrossLevelRight(SwerveDrive swerveDrive, Shoulder shoulder, Arm arm, Claw claw) {
        return scoreCrossLevel(swerveDrive, shoulder, arm, claw, 1);
    }

    public static CommandBase scoreCrossLevelLeft(SwerveDrive swerveDrive, Shoulder shoulder, Arm arm, Claw claw) {
        return scoreCrossLevel(swerveDrive, shoulder, arm, claw, -1);
    }

    public static CommandBase test(SwerveDrive swerveDrive) {
        return new InstantCommand(swerveDrive::resetPose)
            .andThen(new InstantCommand(() -> swerveDrive.resetGyro(90d)))
            .andThen(swerveDrive.driveTo(-55d, 0, 90d, 0.3d, 0.5d))
            .andThen(new LevelChargeStation(swerveDrive));
    }
}
