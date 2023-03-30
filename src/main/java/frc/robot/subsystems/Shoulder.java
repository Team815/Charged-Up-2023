package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase {
    private final CANSparkMax reachMotor;
    private final DutyCycleEncoder encoder;
    private double retracted;

    public Shoulder(int motorId, int encoderChannel) {
        reachMotor = new CANSparkMax(motorId, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = new DutyCycleEncoder(encoderChannel);
        resetPosition();
    }

    public void resetPosition() {
        retracted = getPosition();
    }

    public void periodic() {
        //System.out.println("Shoulder: " + getPosition());
    }

    public void set(double output) {
        reachMotor.set(output);
    }

    public double getPosition() {
        return encoder.get();
    }

    public double getRetractedPosition() {
        return retracted;
    }
}
