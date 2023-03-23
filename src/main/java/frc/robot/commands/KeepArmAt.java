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
    private final double feedForward;

    public static final double FarConeNode = -0.120d;
    public static final double NearConeNode = -0.08d;
    public static final double Substation = -0.07d;
    public static final double AboveFloor = 0.11d;
    public static final double NoConeFf = 0.09d;
    public static final double ConeFf = 0.12d;
    public static final double ConeGroundFf = 0.05d;

    public KeepArmAt(Arm arm, double target, double maxSpeed, double feedForward) {
        super();
        this.arm = arm;
        pid = new PIDController(2d, 0d, 0d);
        this.target = target;
        this.maxSpeed = maxSpeed;
        this.feedForward = feedForward;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        pid.setSetpoint(target);
    }

    @Override
    public void execute() {
        var position = arm.getPosition();
        var response = MathUtil.clamp(-pid.calculate(position), -maxSpeed, maxSpeed) + feedForward;
        arm.set(response);
        System.out.println("Arm: " + position + " -> " + pid.getSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        arm.set(0);
    }

    protected boolean atSetpoint() {
        return pid.atSetpoint();
    }
}
