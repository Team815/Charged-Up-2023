package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface InputDevice {
    double DEFAULT_MAX_HORIZONTAL_SPEED = 1d;
    double DEFAULT_MAX_VERTICAL_SPEED = 1d;
    double DEFAULT_MAX_ANGULAR_SPEED = 1d;

    double getHorizontalSpeed();

    double getVerticalSpeed();

    double getAngularSpeed();

    Trigger resetHeading();

    Trigger cycleLimelightTarget();

    Trigger centerOnTarget();

    void setMaxHorizontalSpeed(double maxHorizontalSpeed);

    void setMaxVerticalSpeed(double maxVerticalSpeed);

    void setMaxAngularSpeed(double maxAngularSpeed);
}
