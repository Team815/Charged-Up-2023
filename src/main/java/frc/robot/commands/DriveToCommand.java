package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DriveToCommand extends CommandBase {

    private SwerveDrive swerveDrive;
    private Pose2d target;
    private PIDController pidController;

    public DriveToCommand(Pose2d target, SwerveDrive swerveDrive) {
        super();
        this.target = target;
        this.swerveDrive = swerveDrive;
        pidController = new PIDController(0.04, 0, 0);
        pidController.setTolerance(0.5);
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        final double maxSpeed = 0.1;
        Pose2d pose = swerveDrive.getPose();
        Translation2d difference = target.minus(swerveDrive.getPose()).getTranslation();
        double response = MathUtil.clamp(Math.abs(pidController.calculate(difference.getNorm())), 0, maxSpeed);
        Translation2d speed = new Translation2d(response, difference.getAngle().rotateBy(pose.getRotation()));
        swerveDrive.drive(speed.getX(), speed.getY(), 0);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
