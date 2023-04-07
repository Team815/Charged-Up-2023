package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BooleanQueue;
import frc.robot.GamePieceDetector;

public class Claw extends SubsystemBase {
    private final Compressor compressor;
    private final Solenoid solenoid;
    private final GamePieceDetector detector = new GamePieceDetector();
    //private final BooleanQueue<Boolean> isDetecting = new BooleanQueue<>(2);
    private boolean isDetecting;

    public Claw(Compressor compressor, Solenoid solenoid) {
        this.compressor = compressor;
        this.solenoid = solenoid;
    }

    @Override
    public void periodic() {
//        isDetecting.add(isDetectingPrivate());
        isDetecting = isDetectingPrivate();
    }

    public void open() {
        solenoid.set(true);
    }

    public void close() {
        System.out.println("Closing claw");
        solenoid.set(false);
    }

    public boolean isOpen() {
        return solenoid.get();
    }

    public boolean isDetecting() {
//        return !isDetecting.allFalse();
        return isDetecting;
    }

    private boolean isDetectingPrivate() {
        final int minRange = 60;
        final int maxRange = 180;
        int reading = detector.detect().orElse(-1);
        return reading >= minRange && reading <= maxRange;
    }
}
