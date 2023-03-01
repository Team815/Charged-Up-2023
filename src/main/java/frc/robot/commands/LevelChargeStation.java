package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class LevelChargeStation extends CommandBase {

    private final SwerveDrive swerveDrive;
    private final DoubleQueue levels;

    public LevelChargeStation(SwerveDrive swerveDrive) {
        super();
        final int queueCapacity = 10;
        this.swerveDrive = swerveDrive;
        levels = new DoubleQueue(queueCapacity);
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        final var threshold = 8d;
        var level = swerveDrive.getLevel();
        if (levels.isFull()) {
            var range = levels.range();
            var largestLevel = Math.abs(range.getFirst()) > Math.abs(range.getSecond()) ? range.getFirst() : range.getSecond();
            var speed = range.getSecond() - range.getFirst() > 2 ? 0d :
                Math.abs(largestLevel) < threshold ? 0d :
                    0.1d * -Math.signum(largestLevel);
            swerveDrive.drive(speed, 0d, 0d, 0.5d);
        }
        levels.add(level);
    }
}
