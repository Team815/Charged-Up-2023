package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class GamePieceLimelight extends Limelight {

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
        setPipeline(target.pipeline);
        var tab = Shuffleboard.getTab("SmartDashboard");
        var layout = tab.getLayout("Teleop", BuiltInLayouts.kGrid);
        layout
            .addString("Limelight Target", target::toString)
            .withPosition(0, 4);
    }

    public void toggleTarget() {
        target = target == Target.Cube ? Target.Cone :
            target == Target.Cone ? Target.ConeNode :
                Target.Cube;
        setPipeline(target.pipeline);
    }
}
