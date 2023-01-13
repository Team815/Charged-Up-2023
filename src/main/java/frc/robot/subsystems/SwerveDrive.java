// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  private SwerveModule moduleFrontLeft;
  private SwerveModule moduleFrontRight;
  private SwerveModule moduleBackLeft;
  private SwerveModule moduleBackRight;
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
      System.out.println("Speed X: " + speedX + " Speed Y: " + speedY + " Rotation Speed: " + rotation);
  }
}
