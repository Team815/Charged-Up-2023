package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;

public record SwerveDriveModule(SwerveModule module, Translation2d translation) {
    public SwerveDriveModule(SwerveModule module, double x, double y) {
        this(module, new Translation2d(x, y));
    }
}
