package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrive;

public class RobotCommander {
    private final SwerveDrive swerveDrive;
    public RobotCommander(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    // Swerve Drive Commands

    public DriveToCommand driveTo(
        double forwardPosition,
        double sidewaysPosition,
        double angularPosition,
        double maxLinearSpeed,
        double maxAngularSpeed) {
        return new DriveToCommand(
            new Pose2d(forwardPosition, sidewaysPosition, Rotation2d.fromDegrees(angularPosition)),
            maxLinearSpeed,
            maxAngularSpeed,
            swerveDrive);
    }

    public LevelChargeStation level() {
        return new LevelChargeStation(swerveDrive);
    }

    public InstantCommand resetPose() {
        return new InstantCommand(swerveDrive::resetPose);
    }

    public InstantCommand resetGyro(double offset) {
        return new InstantCommand(() -> swerveDrive.resetGyro(offset));
    }

}
