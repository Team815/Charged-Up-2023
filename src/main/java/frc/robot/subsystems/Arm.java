package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final MotorControllerGroup liftMotors;
    private final RelativeEncoder encoder;

    //Position Estimates

    @Override
    public void periodic() {
    }

    public Arm(int motor1Index, int motor2Index) {
        var liftMotor1 = new CANSparkMax(motor1Index, CANSparkMaxLowLevel.MotorType.kBrushless);
        var liftMotor2 = new CANSparkMax(motor2Index, CANSparkMaxLowLevel.MotorType.kBrushless);
        liftMotor1.restoreFactoryDefaults();
        liftMotor2.restoreFactoryDefaults();
        liftMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        liftMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        liftMotor1.setInverted(true);
        encoder = liftMotor2.getEncoder();
        liftMotors = new MotorControllerGroup(liftMotor1, liftMotor2);
    }

    public void set(double output) {
        liftMotors.set(output);
    }

    public void zeroEncoder() {
        encoder.setPosition(0d);
    }

    public double getPosition() {
        return encoder.getPosition();
    }
}
