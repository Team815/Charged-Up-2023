package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.TalonSrxAdapter;

public class Arm extends SubsystemBase {

    enum ReachDirection {
        Forward(),
        Reverse();
    }

    private Timer timer;
    private TalonSRX liftMotor1;
    private TalonSRX liftMotor2;
    private TalonSRX reachMotor;
    private MotorControllerGroup liftMotors;
    private PIDController pid;
    private double reachTargetPos;
    private boolean motorsEnabled = false;
    private ReachDirection reachDirection;

    //Position Estimates
    final double topConeV = 974;
    final double topConeH = 18774;
    final double bottomConeV = 774;
    final double bottomConeH = 1177;
    final double stationPickupV = 812;
    final double stationPickupH = 33;
    final double groundPickupV = 55;
    final double groundPickupH = 9260;

    public Arm(int motor1Index, int motor2Index, int motor3Index) {
        liftMotor1 = new TalonSRX(motor1Index);
        liftMotor2 = new TalonSRX(motor2Index);
        liftMotor2.setInverted(true);
        reachMotor = new TalonSRX(motor3Index);

        liftMotors = new MotorControllerGroup (
            new TalonSrxAdapter(liftMotor1, ControlMode.PercentOutput),
            new TalonSrxAdapter(liftMotor2, ControlMode.PercentOutput));

        timer = new Timer();
    }

    public void setToTopCone() {
        //liftMotors.set(1);
        reachTargetPos = 5000;
        reachDirection = ReachDirection.Forward;
        motorsEnabled = true;
    }

    public void setToBottomCone() {
        //liftMotors.set(.75);

        //Temporary code so the motor is only active when a button is pushed
        //This code should be moved to setToHome(),
        //to allow the arm reach to return to it's base position
        reachTargetPos = 0;
        reachDirection = ReachDirection.Reverse;
        motorsEnabled = true;
    }

    public void setToStationPickup() {
        //liftMotors.set(.8);
    }

    public void setToGroundPickup() {
        //liftMotors.set(.3);
    }

    public void setToHome() {
        motorsEnabled = false;
        liftMotors.set(0);
        reachMotor.set(ControlMode.PercentOutput, 0);
    }

    public void periodic() {
        super.periodic();

        var liftCurrentPos = liftMotor1.getSelectedSensorPosition();
        var reachCurrentPos = reachMotor.getSelectedSensorPosition();

        // Move the reach motor to a desired position
        if (motorsEnabled &&
            (reachDirection == ReachDirection.Forward && reachCurrentPos < reachTargetPos ||
                (reachDirection == ReachDirection.Reverse && reachCurrentPos > reachTargetPos))) {
                EnableReachMotor();
        }
        else {
            DisableReachMotor();
        }

        System.out.println("Vertical Rotation: " + liftCurrentPos);
        System.out.println("Horizontal Rotation: " + reachCurrentPos);
    }

    private void EnableReachMotor() {
        if (reachDirection == ReachDirection.Forward) {
            reachMotor.setInverted(false);
            reachMotor.set(ControlMode.PercentOutput, 0.4);
        }
        else {
            reachMotor.setInverted(true);
            reachMotor.set(ControlMode.PercentOutput, 0.4);
        }
    }

    private void DisableReachMotor() {
        reachMotor.set(ControlMode.PercentOutput, 0);
    }
}
