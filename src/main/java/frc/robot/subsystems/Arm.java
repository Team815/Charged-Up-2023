package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Arm extends SubsystemBase {

    private TalonSRX motor1;
    private TalonSRX motor2;
    private TalonSRX motor3;

    private final double homeH = 0d;
    private final double homeV = 0d;
    private final double topConeH = 974d;
    private final double topConeV = 18774d;
    private final double bottomConeH = 774d;
    private final double bottomConeV = 1177d;
    private final double groundPickupH = 55d;
    private final double groundPickupV = 9260d;
    private final double stationPickupH = 812d;
    private final double stationPickupV = 33d;

    public Arm(int motor1Index, int motor2Index, int motor3Index) {
        motor1 = new TalonSRX(motor1Index);
        motor2 = new TalonSRX(motor2Index);
        motor3 = new TalonSRX(motor3Index);
    }

    public void periodic() {
        super.periodic();

        var verticalPos = motor1.getSelectedSensorPosition();
        var horizontalPos = motor3.getSelectedSensorPosition();

        System.out.println("Vertical Rotation: " + verticalPos);
        System.out.println("Horizontal Rotation: " + horizontalPos);
    }
}
