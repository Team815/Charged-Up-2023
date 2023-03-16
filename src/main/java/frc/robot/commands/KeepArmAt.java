package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class KeepArmAt extends CommandBase {
    private final Arm arm;
    private final PIDController pid;
    private final double target;
    private final double maxSpeed;

    public KeepArmAt(Arm arm, double target, double maxSpeed) {
        super();
        this.arm = arm;
        pid = new PIDController(0.1d, 0d, 0d);
        this.target = target;
        this.maxSpeed = maxSpeed;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        pid.setSetpoint(target);
    }

    @Override
    public void execute() {
        var position = arm.getPosition();
        var response = MathUtil.clamp(pid.calculate(position), -maxSpeed, maxSpeed) + 0.08;
        arm.set(response);
    }

    @Override
    public void end(boolean interrupted) {
        arm.set(0);
    }

    protected boolean atSetpoint() {
        return pid.atSetpoint();
    }
}
