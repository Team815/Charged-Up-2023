package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;

public class MoveShoulder extends CommandBase {
    private final Shoulder shoulder;
    private final PIDController pid;
    private final Position target;
    private static final double NEAR_CONE_OFFSET = 0d;
    private static final double FAR_CONE_OFFSET = -4.5d;
    private static final double SUBSTATION_OFFSET = -3.5d;

    public enum Position {
        RETRACTED(0d),
        NEAR_CONE(NEAR_CONE_OFFSET),
        FAR_CONE(FAR_CONE_OFFSET),
        PICKUP(SUBSTATION_OFFSET);

        private final double offset;

        Position(double offset) {
            this.offset = offset;
        }

        public double getOffset() {
            return offset;
        }
    }

    public MoveShoulder(Shoulder shoulder, Position target) {
        super();
        this.shoulder = shoulder;
        pid = new PIDController(2d, 0d, 0d);
        pid.setTolerance(0.01d);
        this.target = target;
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setSetpoint(shoulder.getRetractedPosition() + target.getOffset());
    }

    @Override
    public void execute() {
        final double maxSpeed = .9d;
        var position = shoulder.getPosition();
        var pidValue = MathUtil.clamp(pid.calculate(position), -maxSpeed, maxSpeed);
        shoulder.set(pidValue);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            System.out.println("MoveShoulder finished");
        }
        shoulder.set(0);
    }
}
