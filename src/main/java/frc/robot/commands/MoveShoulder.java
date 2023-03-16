package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;

public class MoveShoulder extends CommandBase {
    private final Shoulder shoulder;
    private final PIDController pid;
    private final double target;
    private DoubleQueue buffer;

    public MoveShoulder(Shoulder shoulder, double target) {
        super();
        this.shoulder = shoulder;
        pid = new PIDController(0.0002d, 0d, 0d);
        pid.setTolerance(100);
        buffer = new DoubleQueue(4);
        this.target = target;
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        buffer = new DoubleQueue(8);
        pid.reset();
        pid.setSetpoint(target);
    }

    @Override
    public void execute() {
        var position = shoulder.getPosition();
        var pidValue = MathUtil.clamp(pid.calculate(position), -0.4d, 0.4d);
        buffer.add(position);
        shoulder.set(pidValue);
    }

    @Override
    public boolean isFinished() {
        var range = buffer.range();
        return buffer.isFull() &&
            range.getSecond() - range.getFirst() <= 1 &&
            Math.abs(range.getFirst() - pid.getSetpoint()) < 200;
    }

    @Override
    public void end(boolean interrupted) {
        shoulder.set(0);
    }
}
