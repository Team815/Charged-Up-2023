package frc.robot;

import java.util.function.Supplier;

public class GamePieceLimelight extends Limelight {

    public enum Target {
        Cube(0),
        Cone(1),
        ConeNode(2);

        final int pipeline;

        Target(int pipeline) {
            this.pipeline = pipeline;
        }
    }

    static Target target = Target.Cone;
    public static Supplier<String> currentTarget = () -> target.toString();

    public GamePieceLimelight(String instance) {
        super(instance);
        setPipeline(target.pipeline);
    }

    public void toggleTarget() {
        target = target == Target.Cube ? Target.Cone :
            target == Target.Cone ? Target.ConeNode :
                Target.Cube;
        setPipeline(target.pipeline);
        currentTarget = () -> target.toString();
    }
}
