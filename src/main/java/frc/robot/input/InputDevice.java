package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface InputDevice {
    double getHorizontalSpeed();

    double getVerticalSpeed();

    double getAngularSpeed();

    Trigger resetHeading();

    Trigger cycleLimelightTarget();

    Trigger centerOnTarget();

    Trigger openClaw();
    Trigger setArmToTopCone();
    Trigger setArmToBottomCone();
    Trigger setArmToStationPickup();
    Trigger setArmToGroundPickup();
}
