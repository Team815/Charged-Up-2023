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
        pid = new PIDController(0.04, 0, 0);
        pid.setTolerance(0.5);
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        final var maxSpeed = 0.1;
        var pose = swerveDrive.getPose();
        var difference = target.minus(swerveDrive.getPose()).getTranslation();
        var response = MathUtil.clamp(Math.abs(pid.calculate(difference.getNorm())), 0, maxSpeed);
        var speed = new Translation2d(response, difference.getAngle().rotateBy(pose.getRotation()));
        swerveDrive.drive(speed.getX(), speed.getY(), 0);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
