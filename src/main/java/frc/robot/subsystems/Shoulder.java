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

    public static final double FAR_CONE_OFFSET = 4.5d;
    public static final double NEAR_CONE_OFFSET = 0d;
    public static final double PICKUP_CONE_OFFSET = 3.5d;
    public static double Retracted = 0.676d;
    public static double FarCone = Retracted - FAR_CONE_OFFSET;
    public static double NearCone = Retracted - NEAR_CONE_OFFSET;
    public static double Pickup = Retracted - PICKUP_CONE_OFFSET;

    public Shoulder(int motorId, int encoderChannel) {
        reachMotor = new CANSparkMax(motorId, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = new DutyCycleEncoder(encoderChannel);
        ResetPositions();
    }

    public void ResetPositions() {
        Retracted = getPosition();
        FarCone = Retracted - FAR_CONE_OFFSET;
        NearCone = Retracted - NEAR_CONE_OFFSET;
        Pickup = Retracted - PICKUP_CONE_OFFSET;
        System.out.println("Shoulder Reset");
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
}
