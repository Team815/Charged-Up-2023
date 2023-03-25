package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DriveTo extends CommandBase {

    private final SwerveDrive swerveDrive;
    private final Pose2d target;
    private final PIDController linearPid;
    private final PIDController angularPid;
    private final double maxLinearSpeed;
    private final double maxAngularSpeed;

    public DriveTo(Pose2d target, double maxLinearSpeed, double maxAngularSpeed, SwerveDrive swerveDrive) {
        super();
        this.target = target;
        this.swerveDrive = swerveDrive;
        linearPid = new PIDController(0.03d, 0.001d, 0d);
        linearPid.setTolerance(0.5d);
        angularPid = new PIDController(0.01d, 0d, 0d);
        angularPid.enableContinuousInput(0d, 360d);
        angularPid.setTolerance(4d);
        angularPid.setSetpoint(target.getRotation().getDegrees());
        this.maxLinearSpeed = Math.abs(maxLinearSpeed);
        this.maxAngularSpeed = Math.abs(maxAngularSpeed);
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
        var linearResponse = Math.min(-linearPid.calculate(difference.getNorm()), maxLinearSpeed);
        var linearSpeed = new Translation2d(linearResponse, difference.getAngle().rotateBy(pose.getRotation()));
        var angularResponse = MathUtil.clamp(angularPid.calculate(swerveDrive.getAngles().getYaw()), -maxAngularSpeed, maxAngularSpeed);
        swerveDrive.drive(linearSpeed.getX(), linearSpeed.getY(), angularResponse);
    }

    @Override
    public boolean isFinished() {
        return linearPid.atSetpoint();// && angularPid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            System.out.println("DriveTo finished");
        }
    }
}
