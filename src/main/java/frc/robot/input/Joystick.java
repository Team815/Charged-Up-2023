package frc.robot.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Joystick extends CommandJoystick implements InputDevice {
    private static final double DEADBAND = 0.15;

    public Joystick() {
        super(0);
    }

    @Override
    public double getSidewaysVelocity() {
        return MathUtil.applyDeadband(-getX(), DEADBAND);
    }

    @Override
    public double getForwardVelocity() {
        return MathUtil.applyDeadband(-getY(), DEADBAND);
    }

    @Override
    public double getAngularVelocity() {
        return MathUtil.applyDeadband(getTwist(), DEADBAND);
    }

    @Override
    public Trigger resetHeading() {
        return button(11);
    }

    @Override
    public Trigger cycleLimelightTarget() {
        return button(2);
    }

    @Override
    public Trigger centerOnTarget() {
        return button(1);
    }

    @Override
    public Trigger openClaw() {
        return null;
    }

    @Override
    public Trigger setArmToTopCone() {
        return povUp();
    }

    @Override
    public Trigger setArmToBottomCone() {
        return povDown();
    }

    @Override
    public Trigger setArmToStationPickup() {
        return povRight();
    }

    @Override
    public Trigger turtle() {
        return povLeft();
    }

    @Override
    public Trigger slow() {
        return null;
    }

    @Override
    public void setMaxSidewaysSpeed(double maxSidewaysSpeed) {
    }

    @Override
    public void setMaxForwardSpeed(double maxForwardSpeed) {
    }

    @Override
    public void setMaxAngularSpeed(double maxAngularSpeed) {
    }
}
