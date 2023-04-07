package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    public static final double RETRACTED = 0.462d;
    private final MotorControllerGroup liftMotors;
    private final DutyCycleEncoder encoder;

    public Arm(int motor1Index, int motor2Index) {
        var liftMotor1 = new CANSparkMax(motor1Index, CANSparkMaxLowLevel.MotorType.kBrushless);
        var liftMotor2 = new CANSparkMax(motor2Index, CANSparkMaxLowLevel.MotorType.kBrushless);
        liftMotor1.restoreFactoryDefaults();
        liftMotor2.restoreFactoryDefaults();
        liftMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        liftMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        liftMotor1.setInverted(true);
        liftMotors = new MotorControllerGroup(liftMotor1, liftMotor2);
        encoder = new DutyCycleEncoder(2);
    }

    public void set(double output) {
        System.out.println(output);
        liftMotors.set(output);
    }

    public void reset() {
        encoder.reset();
        encoder.setPositionOffset(RETRACTED);
    }

    public double getPower() {
        return liftMotors.get();
    }

    public double getPosition() {
        return encoder.get();
    }
}
