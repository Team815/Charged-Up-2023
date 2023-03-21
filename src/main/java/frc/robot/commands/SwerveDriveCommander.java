package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.SwerveDrive;

public class SwerveDriveCommander {
    private final SwerveDrive swerveDrive;

    public SwerveDriveCommander(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

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
}
