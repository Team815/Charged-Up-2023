package frc.robot.subsystems;

import static java.lang.System.nanoTime;

public class Timer {
    private long length;
    private long start;

    public void start(double length) {
        this.length = (long)(length * 1000000000);
        start = nanoTime();
    }

    public boolean isRunning() {
        return (nanoTime() - start) < length;
    }
}
