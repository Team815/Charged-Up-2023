// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class SwerveDriveTest extends SubsystemBase {
  
  private final SwerveModuleTest frontLeft = new SwerveModuleTest(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModuleTest frontRight = new SwerveModuleTest(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModuleTest backLeft = new SwerveModuleTest(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModuleTest backRight = new SwerveModuleTest(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final Pigeon2 gyro = new Pigeon2(0);  

    private final double kWheelBaseX = 0.368;
    private final double kWheelBaseY = 0.368;
    private final Translation2d frontLeftLocation = new Translation2d(kWheelBaseX, -kWheelBaseY);
    private final Translation2d frontRightLocation = new Translation2d(kWheelBaseX, kWheelBaseY);
    private final Translation2d backLeftLocation = new Translation2d(-kWheelBaseX, -kWheelBaseY);
    private final Translation2d backRightLocation = new Translation2d(-kWheelBaseX, kWheelBaseY);
    
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    );

    public final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        kDriveKinematics, getGyroAngle(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }, new Pose2d(5.0, 13.5, new Rotation2d()));
                
  /** Creates a new SwerveDrive. */
  public SwerveDriveTest() {
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    new Thread(() -> {
        try {
            Thread.sleep(1000);
            resetGyro();
        } catch (Exception e) {
        }
    }).start();
  }

  public Rotation2d getGyroAngle(){
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  public void resetGyro() {
    gyro.setYaw(0);
  }

  public Pose2d getPose() {
      return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
      odometer.resetPosition(getGyroAngle(), new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      }, pose);
  }

  @Override
  public void periodic() {
      odometer.update(getGyroAngle(), new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      });
      SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
  }

  public void stopModules() {
      frontLeft.stop();
      frontRight.stop();
      backLeft.stop();
      backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
      //SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
      frontLeft.setDesiredState(desiredStates[0]);
      frontRight.setDesiredState(desiredStates[1]);
      backLeft.setDesiredState(desiredStates[2]);
      backRight.setDesiredState(desiredStates[3]);
  }

  public void drive(double speedX, double speedY, double rotation) {
    drive(speedX, speedY, rotation, false);
  } 

  public void drive(double speedX, double speedY, double rotation, Boolean fieldRelativeSpeed) {
        // 1. Get real-time joystick inputs
        double xSpeed = speedX;
        double ySpeed = speedY;
        double turningSpeed = rotation;

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldRelativeSpeed) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, getGyroAngle());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        setModuleStates(moduleStates);
    }    
}
