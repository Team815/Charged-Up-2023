package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DoubleQueue;
import frc.robot.subsystems.SwerveDrive;

public class LevelChargeStation extends CommandBase {

    private final SwerveDrive swerveDrive;
    private final DoubleQueue levels;
    private double initialAngle;

    private static final double initialSpeed = 0.1d;
    private static final double subsequentSpeed = 0.07d;

    public LevelChargeStation(SwerveDrive swerveDrive) {
        super();
        final int queueCapacity = 8;
        this.swerveDrive = swerveDrive;
        levels = new DoubleQueue(queueCapacity);
        initialAngle = 0d;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        final var threshold = 4d;
        final var maxVariance = 0.2d;
        var level = swerveDrive.getAngles().getRoll();
        if (levels.isFull()) {
            var range = levels.range();
            var largestLevel = Math.abs(range.getFirst()) > Math.abs(range.getSecond()) ? range.getFirst() : range.getSecond();
            if (initialAngle == 0) {
                initialAngle = Math.signum(largestLevel);
            }
            double speed;
//            System.out.println(range.getSecond() - range.getFirst());
            if (range.getSecond() - range.getFirst() > maxVariance || Math.abs(largestLevel) < threshold) {
                speed = 0;
            } else if (initialAngle == Math.signum(largestLevel)) {
                speed = initialSpeed * initialAngle;
            } else {
                initialAngle = 2;
                speed = subsequentSpeed * Math.signum(largestLevel);
            }
            swerveDrive.drive(-speed, 0d, 0d);
        }
        levels.add(level);
    }
}
