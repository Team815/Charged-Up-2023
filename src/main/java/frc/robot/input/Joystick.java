package frc.robot.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Joystick implements InputDevice {
    private static final double DEADBAND = 0.15;
    private final CommandJoystick joystick = new CommandJoystick(0);

    @Override
    public double getHorizontalSpeed() {
        return MathUtil.applyDeadband(-joystick.getX(), DEADBAND);
    }

    @Override
    public double getVerticalSpeed() {
        return MathUtil.applyDeadband(-joystick.getY(), DEADBAND);
    }

    @Override
    public double getAngularSpeed() {
        return MathUtil.applyDeadband(joystick.getTwist(), DEADBAND);
    }

    @Override
    public Trigger resetHeading() {
        return joystick.button(7);
    }
}
