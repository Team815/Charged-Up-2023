package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.EnumSet;
import java.util.Map;

public class GamePieceLimelight extends Limelight {
    public static final Target DEFAULT_TARGET = Target.Cone;

    private enum Target {
        Cube(0),
        Cone(1);

        private final int pipeline;

        Target(int pipeline) {
            this.pipeline = pipeline;
        }
    }

    private Target target;

    public GamePieceLimelight(String instance) {
        super(instance);
        setTarget(DEFAULT_TARGET);
    }

    public void cycleTarget() {
        setTarget(target == Target.Cube ? Target.Cone : Target.Cube);
    }

    public String getTarget() {
        return target.toString();
    }

    private void setTarget(Target target) {
        this.target = target;
        setPipeline(target.pipeline);
    }
}
