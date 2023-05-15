package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class RangePidController extends PIDController {
    private final double min;
    private final double max;

    public RangePidController(double kp, double ki, double kd, double min, double max) {
        super(kp, ki, kd);
        this.min = min;
        this.max = max;
    }

    @Override
    public double calculate(double measurement) {
        return MathUtil.clamp(super.calculate(measurement), min, max);
    }

    @Override
    public double calculate(double measurement, double setpoint) {
        return MathUtil.clamp(super.calculate(measurement, setpoint), min, max);
    }
}
