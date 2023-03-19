package frc.robot.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxController extends CommandXboxController implements InputDevice {
    private static final double DEADBAND = 0.15;

    public XboxController() {
        super(0);
    }

    @Override
    public double getHorizontalSpeed() {
        return MathUtil.applyDeadband(-getLeftX(), DEADBAND);
    }

    @Override
    public double getVerticalSpeed() {
        return MathUtil.applyDeadband(-getLeftY(), DEADBAND);
    }

    @Override
    public double getAngularSpeed() {
        return MathUtil.applyDeadband(-getRightX(), DEADBAND);
    }

    @Override
    public Trigger resetHeading() {
        return start();
    }

    @Override
    public Trigger cycleLimelightTarget() {
        return leftBumper();
    }

    @Override
    public Trigger centerOnTarget() {
        return rightBumper();
    }
}
