package frc.robot.commands;

import frc.robot.subsystems.Arm;

public class LiftArmTo extends KeepArmAt {

    public LiftArmTo(Arm arm, double target, double maxSpeed) {
        super(arm, target, maxSpeed);
    }

    @Override
    public boolean isFinished() {
        return atSetpoint();
    }
}
