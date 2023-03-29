package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.SwerveDrive;

public class RobotCommander {
    private final SwerveDrive swerveDrive;
    private final Shoulder shoulder;
    private final Arm arm;
    private final Claw claw;

    public RobotCommander(SwerveDrive swerveDrive, Shoulder shoulder, Arm arm, Claw claw) {
        this.swerveDrive = swerveDrive;
        this.shoulder = shoulder;
        this.arm = arm;
        this.claw = claw;
    }

    // Swerve Drive Commands

    public DriveTo driveTo(
        double forwardPosition,
        double sidewaysPosition,
        double angularPosition,
        double maxLinearSpeed,
        double maxAngularSpeed) {
        return new DriveTo(
            new Pose2d(forwardPosition, sidewaysPosition, Rotation2d.fromDegrees(angularPosition)),
            maxLinearSpeed,
            maxAngularSpeed,
            swerveDrive);
    }

    public LevelChargeStation level() {
        return new LevelChargeStation(swerveDrive);
    }

    public InstantCommand resetPose() {
        return new InstantCommand(swerveDrive::resetPose);
    }

    public InstantCommand resetGyro(double offset) {
        return new InstantCommand(() -> swerveDrive.resetGyro(offset));
    }

    // Shoulder Commands

    public MoveShoulder moveShoulder(double target) {
        return new MoveShoulder(shoulder, target);
    }
    public InstantCommand resetShoulder() { return new InstantCommand(shoulder::ResetPositions); }

    // Arm Commands

    public KeepArmAt keepArmAt(double target, double feedForward) {
        return keepArmAt(target, 0.2d, feedForward);
    }

    public KeepArmAt keepArmAt(double target, double maxSpeed, double feedForward) {
        return new KeepArmAt(arm, target, maxSpeed, feedForward);
    }

    public LiftArmTo liftArmTo(double target, double feedForward) {
        return liftArmTo(target, 0.2d, feedForward);
    }

    public LiftArmTo liftArmTo(double target, double maxSpeed, double feedForward) {
        return new LiftArmTo(arm, target, maxSpeed, feedForward);
    }

    public DropArm dropArm() {
        return new DropArm(arm);
    }

    // Claw Commands

    public InstantCommand openClaw() {
        return new InstantCommand(claw::open);
    }

    public InstantCommand closeClaw() {
        return new InstantCommand(claw::close);
    }
}
