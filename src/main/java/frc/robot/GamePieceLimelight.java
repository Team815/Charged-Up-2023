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

    private double p;
    private GenericEntry pEntry;

    private enum Target {
        Cube(0),
        Cone(1),
        ConeNode(2);

        private final int pipeline;

        Target(int pipeline) {
            this.pipeline = pipeline;
        }
    }

    private Target target = Target.Cone;

    public GamePieceLimelight(String instance) {
        super(instance);
        p = 0.01d;
        setPipeline(target.pipeline);
        var tab = Shuffleboard.getTab("SmartDashboard");
        var layout = tab
            .getLayout("Limelight", BuiltInLayouts.kGrid)
            .withSize(2, 1)
            .withProperties(Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 2));
        layout
            .addString("Target", () -> target.toString())
            .withPosition(0, 0);
        pEntry = layout
            .add("P", p)
            .withPosition(0, 1)
            .getEntry();
        var inst = NetworkTableInstance.getDefault();
        inst.addListener(
            pEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> p = e.valueData.value.getDouble());
    }

    public void cycleTarget() {
        target = target == Target.Cube ? Target.Cone :
            target == Target.Cone ? Target.ConeNode :
                Target.Cube;
        setPipeline(target.pipeline);
    }

    public double getP() {
        return p;
    }
}
