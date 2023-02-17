package frc.robot.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxController implements InputDevice {
    private static final double DEADBAND = 0.15;
    private final CommandXboxController xboxController = new CommandXboxController(0);

    @Override
    public double getHorizontalSpeed() {
        return MathUtil.applyDeadband(-xboxController.getLeftX(), DEADBAND);
    }

    @Override
    public double getVerticalSpeed() {
        return MathUtil.applyDeadband(-xboxController.getLeftY(), DEADBAND);
    }

    @Override
    public double getAngularSpeed() {
        return MathUtil.applyDeadband(xboxController.getRightX(), DEADBAND);
    }

    @Override
    public Trigger resetHeading() {
        return xboxController.start();
    }
}
