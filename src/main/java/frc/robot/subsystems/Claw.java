package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private final Compressor compressor;
    private final Solenoid solenoid;

    public Claw(Compressor compressor, Solenoid solenoid) {
        this.compressor = compressor;
        this.solenoid = solenoid;
    }

    public void open() {
        solenoid.set(true);
    }

    public void close() {
        solenoid.set(false);
    }
}
