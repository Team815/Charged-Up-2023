package frc.robot.subsystems;

public class GyroAngles {
    private double pitch;
    private double roll;
    private double yaw;

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
