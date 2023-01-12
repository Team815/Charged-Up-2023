// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

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

    //private final AHRS gyro = new AHRS(SPI.Port.kMXP);    

    private final Translation2d m_frontLeftLocation = new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2);
    private final Translation2d m_frontRightLocation = new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2);
    private final Translation2d m_backLeftLocation = new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2);
    private final Translation2d m_backRightLocation = new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2);
    
    public final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
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
    new Thread(() -> {
        try {
            Thread.sleep(1000);
            zeroHeading();
        } catch (Exception e) {
        }
    }).start();
  }

  public Rotation2d getGyroAngle(){
    return new Rotation2d();
  }

  public void zeroHeading() {
      //gyro.reset();
  }

  public double getHeading() {
      return Math.IEEEremainder(getGyroAngle().getDegrees(), 360);
  }

  public Rotation2d getRotation2d() {
      return Rotation2d.fromDegrees(getHeading());
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
      odometer.update(getRotation2d(), new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      });
      SmartDashboard.putNumber("Robot Heading", getHeading());
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
}
