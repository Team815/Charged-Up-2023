// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  private final Pigeon2 gyro;
  private final SwerveModule moduleFrontLeft;
  private final SwerveModule moduleFrontRight;
  private final SwerveModule moduleBackLeft;
  private final SwerveModule moduleBackRight;
  private final SwerveDriveKinematics kinematics;

  /** Creates a new SwerveDrive. */
  public SwerveDrive(
    SwerveModule moduleFrontLeft,
    SwerveModule moduleFrontRight,
    SwerveModule moduleBackLeft,
    SwerveModule moduleBackRight) {
      gyro = new Pigeon2(0);
      this.moduleFrontLeft = moduleFrontLeft;
      this.moduleFrontRight = moduleFrontRight;
      this.moduleBackLeft = moduleBackLeft;
      this.moduleBackRight = moduleBackRight;
      final double x = 0.368;
      final double y = 0.368;
      kinematics = new SwerveDriveKinematics(
        new Translation2d(x, -y),
        new Translation2d(-x, -y),
        new Translation2d(x, y),
        new Translation2d(-x, y));
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double speedX, double speedY, double rotation) {
      speedX = cleanInput(speedX);
      speedY = cleanInput(speedY);
      rotation = cleanInput(rotation);

      double speed = Math.sqrt(speedX * speedX + speedY * speedY);
      double angle = speedX != 0 ? Math.toDegrees(Math.atan(speedY / speedX)) : speedY > 0 ? 90 : -90;
      if (speedX < 0) {
        angle += 180;
      }
      angle -= gyro.getYaw() + 90;

      double speedX1 = speed * Math.cos(Math.toRadians(angle));
      double speedY1 = speed * Math.sin(Math.toRadians(angle));

      System.out.printf("xStart: %.2f, yStart: %.2f, xEnd: %.2f, yEnd: %.2f\n", speedX, speedY, speedX1, speedY1);
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(speedX1, speedY1, rotation));

      for(SwerveModuleState state : states){
        state.speedMetersPerSecond /= 8;
      }

      moduleFrontLeft.drive(states[0].speedMetersPerSecond, states[0].angle.getDegrees());
      moduleFrontRight.drive(states[1].speedMetersPerSecond, states[1].angle.getDegrees());
      moduleBackLeft.drive(states[2].speedMetersPerSecond, states[2].angle.getDegrees());
      moduleBackRight.drive(states[3].speedMetersPerSecond, states[3].angle.getDegrees());
      System.out.println("Speed: " + states[0].speedMetersPerSecond + ", rotation: " + states[0].angle.getDegrees());
  }

  private static double cleanInput(double input) {
      final double deadzone = 0.15;
      double multiple = 1 / (1 - deadzone);
      double cleanedInput = (Math.abs(input) - deadzone) * multiple;
      return Math.max(0, cleanedInput) * Math.signum(input);
  }
}
