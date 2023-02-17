package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DriveToCommand extends CommandBase {

    private final SwerveDrive swerveDrive;
    private final Pose2d target;
    private final PIDController pid;

    public DriveToCommand(Pose2d target, SwerveDrive swerveDrive) {
        super();
        this.target = target;
        this.swerveDrive = swerveDrive;
        pid = new PIDController(0.04d, 0d, 0d);
        pid.setTolerance(0.5d);
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        final var maxSpeed = 0.1d;
        var pose = swerveDrive.getPose();
        var difference = target.minus(swerveDrive.getPose()).getTranslation();
        var response = MathUtil.clamp(Math.abs(pid.calculate(difference.getNorm())), 0d, maxSpeed);
        var speed = new Translation2d(response, difference.getAngle().rotateBy(pose.getRotation()));
        swerveDrive.drive(speed.getX(), speed.getY(), 0d);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
