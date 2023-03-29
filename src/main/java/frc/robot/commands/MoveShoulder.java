package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;

public class MoveShoulder extends CommandBase {
    private final Shoulder shoulder;
    private final PIDController pid;
    private final double target;

    public MoveShoulder(Shoulder shoulder, double target) {
        super();
        this.shoulder = shoulder;
        pid = new PIDController(2d, 0d, 0d);
        pid.setTolerance(0.01d);
        this.target = target;
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setSetpoint(target);
    }

    @Override
    public void execute() {
        final double maxSpeed = .9d;
        var position = shoulder.getPosition();
        var pidValue = MathUtil.clamp(pid.calculate(position), -maxSpeed, maxSpeed);
        shoulder.set(pidValue);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            System.out.println("MoveShoulder finished");
        }
        shoulder.set(0);
    }
}
