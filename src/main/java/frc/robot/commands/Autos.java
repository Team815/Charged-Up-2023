// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Shoulder;

public final class Autos {
    /**
     * Example static factory for an autonomous command.
     */

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static CommandBase score(RobotCommander commander) {
        return commander.resetPose()
            .andThen(
                commander.resetGyro(180d),
                commander.driveTo(4d, 0d, 180d, 0.2d, 0.2d)
                    .deadlineWith(commander.keepArmAt(KeepArmAt.FarConeNode, KeepArmAt.FarConeNodeFf)),
                commander.moveShoulder(MoveShoulder.Position.FAR_CONE)
                    .alongWith(commander.driveTo(0d, 0d, 180d, 0.2d, 0.2d))
                    .deadlineWith(commander.keepArmAt(KeepArmAt.FarConeNode, KeepArmAt.FarConeNodeFf)))
            .withTimeout(4d)
            .andThen(new WaitCommand(0.3d)
                .deadlineWith(
                    commander.openClaw(),
                    commander.keepArmAt(KeepArmAt.FarConeNode, KeepArmAt.NoConeFf)));
    }

    private static CommandBase scoreCross(RobotCommander commander) {
        return score(commander)
            .andThen(commander.moveShoulder(MoveShoulder.Position.RETRACTED)
                .alongWith(
                    commander.driveTo(75d, 0d, 180d, 0.6d, 0.5d),
                    commander.closeClaw(),
                    new WaitCommand(0.3d)
                        .deadlineWith(commander.keepArmAt(KeepArmAt.FarConeNode, KeepArmAt.NoConeFf))
                        .andThen(commander.dropArm())));
    }

    private static CommandBase scoreCrossLevel(RobotCommander commander, int direction) {
        final double chargeStationPositionX = 40d;
        final double chargeStationPositionY = 45d;
        return scoreCross(commander)
            .andThen(
                commander.driveTo(85d, chargeStationPositionY * Math.signum(direction), 90d, 0.6d, 0.5d),
                commander.driveTo(chargeStationPositionX, chargeStationPositionY * Math.signum(direction), 90d, 0.3d, 0.5d),
                commander.level());
    }

    public static CommandBase scoreCrossLevelRight(RobotCommander commander) {
        return scoreCrossLevel(commander, 1);
    }

    public static CommandBase scoreCrossLevelLeft(RobotCommander commander) {
        return scoreCrossLevel(commander, -1);
    }

    public static CommandBase scoreCrossLevelCenter(RobotCommander commander) {
        return score(commander)
            .andThen(
                commander.moveShoulder(MoveShoulder.Position.RETRACTED)
                .alongWith(
                    commander.closeClaw(),
                    new WaitCommand(0.3d)
                        .deadlineWith(
                            commander.keepArmAt(KeepArmAt.FarConeNode, KeepArmAt.NoConeFf),
                            commander.driveTo(5d, 0d, 180d, 0.6d, 0.5d))
                        .andThen(
                            commander.dropArm()
                                .deadlineWith(commander.driveTo(5d, 0d, 90d, 0.6d, 0.5d)))),
                commander.driveTo(85d, 0d, 90d, 0.25d, 0.5d),
                commander.driveTo(45d, 0d, 90d, 0.3d, 0.5d),
                commander.level());
    }

    public static CommandBase test(RobotCommander commander) {
        return commander.resetPose()
            .andThen(
                commander.resetGyro(90d),
                commander.driveTo(-40d, 0, 90d, 0.2d, 0.5d),
                commander.level());
    }
}
