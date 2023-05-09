package frc.robot.subsystems;

public class GyroAngles {
    private final double pitch;
    private final double roll;
    private final double yaw;

    public GyroAngles(double pitch, double roll, double yaw) {
        this.pitch = pitch;
        this.roll = roll;
        this.yaw = yaw;
    }

    public double getPitch() {
        return pitch;
    }

    public double getRoll() {
        return roll;
    }

    public double getYaw() {
        return yaw;
    }
}
