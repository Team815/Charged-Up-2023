package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConeDetector;

public class Claw extends SubsystemBase {
    private final Compressor compressor;
    private final Solenoid solenoid;
    private final ConeDetector coneDetector;

    public Claw(Compressor compressor, Solenoid solenoid) {
        this.compressor = compressor;
        this.solenoid = solenoid;
        coneDetector = new ConeDetector();
    }

    @Override
    public void periodic() {
        var coneDectected = coneDetector.detect();
        System.out.println(coneDectected.orElse(-1));
    }

    public void open() {
        solenoid.set(true);
    }

    public void close() {
        solenoid.set(false);
    }
}
