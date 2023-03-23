// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.SwerveDrive;

public final class Autos {
    /**
     * Example static factory for an autonomous command.
     */

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static CommandBase score(RobotCommander commander) {
        return commander.resetPose().andThen(
                commander.resetGyro(180d),
                commander.driveTo(4d, 0d, 180d, 0.2d, 0.2d)
                  .withTimeout(3d),
                commander.driveTo(0d, 0d, 180d, 0.2d, 0.2d)
                    .withTimeout(3d));
    }

    public static CommandBase scoreCross(RobotCommander commander) {
        return score(commander)
            .andThen(commander.driveTo(85d, 0d, 180d, 0.6d, 0.5d)
                .withTimeout(3d));
    }

    private static CommandBase scoreCrossLevel(RobotCommander commander, int direction) {
        final double chargeStationPositionX = 30d;
        final double chargeStationPositionY = 41d;
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
                commander.driveTo(0, 0, 90, 0.2, 0.2),
                commander.driveTo(90, 0, 90, .4, .4),
                commander.driveTo(50, 0, 90, .3, .3),
                commander.level());
    }

    public static CommandBase test(RobotCommander commander) {
        return commander.resetPose()
            .andThen(
                commander.resetGyro(90d),
                commander.driveTo(-55d, 0, 90d, 0.3d, 0.5d),
                commander.level());
    }

//    public CommandBase MySwerveControllerCommand() {
//        var config = new TrajectoryConfig(2d, 0.2d).setKinematics(kinematics);
//
//        var exampleTrajectory =
//            TrajectoryGenerator.generateTrajectory(
//                new Pose2d(0, 0, new Rotation2d(0)),
//                List.of(new Translation2d(5, 5), new Translation2d(10, -5)),
//                new Pose2d(10, 0, new Rotation2d(0)),
//                config);
//        var thetaController =
//            new ProfiledPIDController(
//                0.1d, 0, 0, new TrapezoidProfile.Constraints(1d, 1d));
//        thetaController.enableContinuousInput(0, 360);
//
//        System.out.println(exampleTrajectory.getStates().size());
//        for (var state : exampleTrajectory.getStates()) {
//            System.out.println(state.toString());
//        }
//
//        return new SwerveControllerCommand(
//            exampleTrajectory,
//            this::getPose,
//            kinematics,
//            new PIDController(0.1d, 0d, 0d),
//            new PIDController(0.1d, 0d, 0d),
//            thetaController,
//            this::setSwerveModuleStates,
//            this);
//    }
}
