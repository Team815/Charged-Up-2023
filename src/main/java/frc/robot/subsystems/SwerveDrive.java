// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  private final SwerveModule moduleFrontLeft;
  private final SwerveModule moduleFrontRight;
  private final SwerveModule moduleBackLeft;
  private final SwerveModule moduleBackRight;

  /** Creates a new SwerveDrive. */
  public SwerveDrive(
    SwerveModule moduleFrontLeft,
    SwerveModule moduleFrontRight,
    SwerveModule moduleBackLeft,
    SwerveModule moduleBackRight) {
      this.moduleFrontLeft = moduleFrontLeft;
      this.moduleFrontRight = moduleFrontRight;
      this.moduleBackLeft = moduleBackLeft;
      this.moduleBackRight = moduleBackRight;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double speedX, double speedY, double rotation) {
      final double deadzone = 0.15;
      if (Math.abs(speedX) < deadzone) {
          speedX = 0;
      }
      if (Math.abs(speedY) < deadzone) {
          speedY = 0;
      }
      if (Math.abs(rotation) < deadzone) {
          rotation = 0;
      }
      rotation /= 5;
      System.out.println("Speed X: " + speedX + " Speed Y: " + speedY + " Rotation Speed: " + rotation);

      moduleFrontLeft.drive(speedY, rotation);
      moduleFrontRight.drive(speedY, rotation);
      moduleBackLeft.drive(speedY, rotation);
      moduleBackRight.drive(speedY, rotation);
  }
}
