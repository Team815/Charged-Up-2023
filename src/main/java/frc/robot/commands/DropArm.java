package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class DropArm extends CommandBase {
    Arm arm;
    private final PIDController pid;

    public DropArm(Arm arm) {
        this.arm = arm;
        pid = new PIDController(0.5d, 0d, 0d);
        pid.setTolerance(0.02);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        pid.setSetpoint(Arm.RETRACTED);
    }

    @Override
    public void execute() {
        var output = -pid.calculate(arm.getPosition());
//        System.out.println(output);
        arm.set(output);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            System.out.println("DropArm finished");
        }
        arm.set(0d);
    }
}
