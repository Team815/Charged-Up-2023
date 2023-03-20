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
    public double getHorizontalSpeed() {
        return MathUtil.applyDeadband(-getX(), DEADBAND);
    }

    @Override
    public double getVerticalSpeed() {
        return MathUtil.applyDeadband(-getY(), DEADBAND);
    }

    @Override
    public double getAngularSpeed() {
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
    public void setMaxHorizontalSpeed(double maxHorizontalSpeed) {
    }

    @Override
    public void setMaxVerticalSpeed(double maxVerticalSpeed) {
    }

    @Override
    public void setMaxAngularSpeed(double maxAngularSpeed) {
    }
}
