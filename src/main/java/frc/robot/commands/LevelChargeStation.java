package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

public class LevelChargeStation extends CommandBase {

    private final int QueueCapacity = 10;
    private final SwerveDrive swerveDrive;
    private final Queue<Double> levels;

    public LevelChargeStation(SwerveDrive swerveDrive) {
        super();
        this.swerveDrive = swerveDrive;
        levels = new LinkedBlockingQueue<>(QueueCapacity);
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        var level = swerveDrive.getLevel();
        if (levels.size() == QueueCapacity) {
            var speed = Math.abs(level) < 8d ? 0d : 0.1d * -Math.signum(level);
            swerveDrive.drive(speed, 0d, 0d, 0.5d);
        }
        addToQueue(levels, level);
    }

    private static void addToQueue(Queue<Double> queue, double newEntry) {
        if (!queue.offer(newEntry)) {
            queue.remove();
            queue.add(newEntry);
        }
    }
}
