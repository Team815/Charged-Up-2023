package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DriveToCommand extends CommandBase {

    private final SwerveDrive swerveDrive;
    private final Pose2d target;
    private final PIDController pid;
    private final double maxLinearSpeed;
    private final double maxAngularSpeed;

    public DriveToCommand(Pose2d target, double maxLinearSpeed, double maxAngularSpeed, SwerveDrive swerveDrive) {
        super();
        this.target = target;
        this.swerveDrive = swerveDrive;
        pid = new PIDController(0.03d, 0.001d, 0d);
        pid.setTolerance(0.5d);
        this.maxLinearSpeed = maxLinearSpeed;
        this.maxAngularSpeed = maxAngularSpeed;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        swerveDrive.setAngle(target.getRotation().getDegrees());
    }

    @Override
    public void execute() {
        var pose = swerveDrive.getPose();
        var difference = target.minus(swerveDrive.getPose()).getTranslation();
        var response = Math.min(-pid.calculate(difference.getNorm()), maxLinearSpeed);
        var speed = new Translation2d(response, difference.getAngle().rotateBy(pose.getRotation()));
        swerveDrive.drive(speed.getX(), speed.getY(), 0d, maxAngularSpeed);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
