package frc.robot.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxController extends CommandXboxController implements InputDevice {
    private static final double DEADBAND = 0.15d;
    private double scale = 1d;

    public XboxController() {
        super(0);
    }

    @Override
    public double getHorizontalSpeed() {
        return MathUtil.applyDeadband(-getLeftX(), DEADBAND) * scale;
    }

    @Override
    public double getVerticalSpeed() {
        return MathUtil.applyDeadband(-getLeftY(), DEADBAND) * scale;
    }

    @Override
    public double getAngularSpeed() {
        return MathUtil.applyDeadband(getRightX(), DEADBAND) * scale;
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
    public Trigger openClaw() {
        return a();
    }

    @Override
    public Trigger setArmToTopCone() {
        return y();
    }

    @Override
    public Trigger setArmToBottomCone() {
        return b();
    }

    @Override
    public Trigger setArmToStationPickup() {
        return x();
    }

    @Override
    public Trigger turtle() {
        return povDown();
    }

    @Override
    public Trigger slow() {
        return this.leftTrigger();
    }

    @Override
    public void setScale(double scale) {
        this.scale = scale;
    }
}
