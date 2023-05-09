package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface InputDevice {
    double DEFAULT_MAX_SIDEWAYS_SPEED = 1d;
    double DEFAULT_MAX_FORWARD_SPEED = 1d;
    double DEFAULT_MAX_ANGULAR_SPEED = 1d;

    double getSidewaysVelocity();

    double getForwardVelocity();

    double getAngularVelocity();

    Trigger resetHeading();

    Trigger slow();

    void setMaxSidewaysSpeed(double maxSidewaysSpeed);

    void setMaxForwardSpeed(double maxForwardSpeed);

    void setMaxAngularSpeed(double maxAngularSpeed);
}
