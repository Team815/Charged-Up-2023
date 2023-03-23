package frc.robot.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxController extends CommandXboxController implements InputDevice {
    private double maxSidewaysSpeed = DEFAULT_MAX_SIDEWAYS_SPEED;
    private double maxForwardSpeed = DEFAULT_MAX_FORWARD_SPEED;
    private double maxAngularSpeed = DEFAULT_MAX_ANGULAR_SPEED;
    private static final double DEADBAND = 0.15d;

    public XboxController() {
        super(0);
    }

    @Override
    public double getSidewaysVelocity() {
        return MathUtil.applyDeadband(-getLeftX(), DEADBAND) * maxSidewaysSpeed;
    }

    @Override
    public double getForwardVelocity() {
        return MathUtil.applyDeadband(-getLeftY(), DEADBAND) * maxForwardSpeed;
    }

    @Override
    public double getAngularVelocity() {
        return MathUtil.applyDeadband(-getRightX(), DEADBAND) * maxAngularSpeed;
    }

    @Override
    public Trigger resetHeading() {
        return start();
    }

    @Override
    public Trigger cycleLimelightTarget() {
        return rightBumper();
    }

    @Override
    public Trigger centerOnTarget() {
        return leftBumper();
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
    public Trigger test1() {
        return povLeft();
    }

    @Override
    public Trigger test2() {
        return povRight();
    }

    @Override
    public void setMaxSidewaysSpeed(double maxSidewaysSpeed) {
        this.maxSidewaysSpeed = MathUtil.clamp(Math.abs(maxSidewaysSpeed), 0, 1);
    }

    @Override
    public void setMaxForwardSpeed(double maxForwardSpeed) {
        this.maxForwardSpeed = MathUtil.clamp(Math.abs(maxForwardSpeed), 0, 1);
    }

    @Override
    public void setMaxAngularSpeed(double maxAngularSpeed) {
        this.maxAngularSpeed = MathUtil.clamp(Math.abs(maxAngularSpeed), 0, 1);
    }
}
