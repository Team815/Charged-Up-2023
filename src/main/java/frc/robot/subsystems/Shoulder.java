package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase {
    private TalonSRX reachMotor;

    public void periodic() {
//        System.out.println("Shoulder " + getPosition());
    }

    final double topConeH = 18774;
    final double bottomConeH = 1177;
    final double stationPickupH = 33;
    final double groundPickupH = 9260;


    public Shoulder(int motorIndex) {
        reachMotor = new TalonSRX(motorIndex);
    }

    public void set(double output) {
        reachMotor.set(ControlMode.PercentOutput, output);
    }

    public double getPosition() {
        return reachMotor.getSelectedSensorPosition();
    }
}
