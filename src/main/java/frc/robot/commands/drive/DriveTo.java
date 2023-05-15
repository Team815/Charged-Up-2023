package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DriveTo extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final Pose2d targetPose;
    private final PIDController linearPid;
    private final PIDController angularPid;

    public DriveTo(SwerveDrive swerveDrive, Pose2d targetPose, PIDController linearPid, PIDController angularPid) {
        this.swerveDrive = swerveDrive;
        this.targetPose = targetPose;
        this.linearPid = linearPid;
        this.angularPid = angularPid;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        swerveDrive.resetPose();
    }

    @Override
    public void execute() {
        var currentPose = swerveDrive.getPose();
        var delta = targetPose.minus(currentPose).getTranslation();
        var linearResponse = -linearPid.calculate(delta.getNorm());
        var linearSpeed = new Translation2d(linearResponse, delta.getAngle().rotateBy(currentPose.getRotation()));
        var angularResponse = angularPid.calculate(swerveDrive.getAngles().getYaw());
        swerveDrive.drive(linearSpeed.getX(), linearSpeed.getY(), angularResponse);
    }

    @Override
    public boolean isFinished() {
        return linearPid.atSetpoint() && angularPid.atSetpoint();
    }
}
