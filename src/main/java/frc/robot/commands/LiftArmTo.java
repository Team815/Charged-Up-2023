package frc.robot.commands;

import frc.robot.subsystems.Arm;

public class LiftArmTo extends KeepArmAt {

    public LiftArmTo(Arm arm, double target, double maxSpeed, double feedForward) {
        super(arm, target, maxSpeed, feedForward);
    }

    @Override
    public boolean isFinished() {
        return atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            System.out.println("LiftArmTo finished");
        }
    }
}
