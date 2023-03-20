package frc.robot.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxController extends CommandXboxController implements InputDevice {
    private double maxHorizontalSpeed = DEFAULT_MAX_HORIZONTAL_SPEED;
    private double maxVerticalSpeed = DEFAULT_MAX_VERTICAL_SPEED;
    private double maxAngularSpeed = DEFAULT_MAX_ANGULAR_SPEED;
    private static final double DEADBAND = 0.15d;

    public XboxController() {
        super(0);
    }

    @Override
    public double getHorizontalSpeed() {
        return MathUtil.applyDeadband(-getLeftX(), DEADBAND) * maxHorizontalSpeed;
    }

    @Override
    public double getVerticalSpeed() {
        return MathUtil.applyDeadband(-getLeftY(), DEADBAND) * maxVerticalSpeed;
    }

    @Override
    public double getAngularSpeed() {
        return MathUtil.applyDeadband(-getRightX(), DEADBAND) * maxAngularSpeed;
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

    @Override
    public void setMaxHorizontalSpeed(double maxHorizontalSpeed) {
        this.maxHorizontalSpeed = MathUtil.clamp(Math.abs(maxHorizontalSpeed), 0, 1);
    }

    @Override
    public void setMaxVerticalSpeed(double maxVerticalSpeed) {
        this.maxVerticalSpeed = MathUtil.clamp(Math.abs(maxVerticalSpeed), 0, 1);
    }

    @Override
    public void setMaxAngularSpeed(double maxAngularSpeed) {
        this.maxAngularSpeed = MathUtil.clamp(Math.abs(maxAngularSpeed), 0, 1);
    }
}
