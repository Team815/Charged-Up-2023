// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
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
  private final SwerveModule[] modules;
  private final SwerveDriveKinematics kinematics;
  private final PIDController pid = new PIDController(0.01, 0, 0);

  /** Creates a new SwerveDrive. */
  public SwerveDrive(
    SwerveModule moduleFrontLeft,
    SwerveModule moduleFrontRight,
    SwerveModule moduleBackLeft,
    SwerveModule moduleBackRight) {
      gyro = new Pigeon2(0);
      resetGyro();
      this.moduleFrontLeft = moduleFrontLeft;
      this.moduleFrontRight = moduleFrontRight;
      this.moduleBackLeft = moduleBackLeft;
      this.moduleBackRight = moduleBackRight;
      modules = new SwerveModule[] {
        this.moduleFrontLeft,
        this.moduleFrontRight,
        this.moduleBackLeft,
        this.moduleBackRight};
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
      double yaw = gyro.getYaw();

      double speed = Math.sqrt(speedX * speedX + speedY * speedY);
      double angle = speedX != 0 ? Math.toDegrees(Math.atan(speedY / speedX)) : speedY > 0 ? 90 : -90;
      if (speedX < 0) {
        angle += 180;
      }
      angle -= yaw;

      double speedX1 = speed * Math.cos(Math.toRadians(angle));
      double speedY1 = speed * Math.sin(Math.toRadians(angle));

      if (rotation != 0) {
        pid.setSetpoint(yaw);
      } else {
        //rotation -= pid.calculate(yaw);
      }

      SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(speedX1, speedY1, rotation));

      for(int i = 0; i < 4; i++) {
        modules[i].drive(states[i]);
      }
  }

  public void resetGyro() {
    gyro.setYaw(0);
    pid.setSetpoint(0);
  }

  private static double cleanInput(double input) {
      final double deadzone = 0.15;
      double multiple = 1 / (1 - deadzone);
      double cleanedInput = (Math.abs(input) - deadzone) * multiple;
      return Math.max(0, cleanedInput) * Math.signum(input);
  }
}
