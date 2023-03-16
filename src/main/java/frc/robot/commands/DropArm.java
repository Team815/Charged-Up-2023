package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class DropArm extends CommandBase {
    Arm arm;
    private final PIDController pid;
    public static boolean isRunning = false;

    public DropArm(Arm arm) {
        this.arm = arm;
        pid = new PIDController(0.02d, 0d, 0d);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        pid.setSetpoint(0d);
    }

    @Override
    public void execute() {
        isRunning = true;
        arm.set(pid.calculate(arm.getPosition()));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getPosition()) < 0.5d;
    }

    @Override
    public void end(boolean interrupted) {
        arm.set(0d);
        isRunning = false;
    }
}
