package frc.robot;

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

    Target target;

    public GamePieceLimelight(String instance) {
        super(instance);
        target = Target.Cone;
        setPipeline(target.pipeline);
    }

    public void toggleTarget() {
        target = target == Target.Cube ? Target.Cone :
            target == Target.Cone ? Target.ConeNode :
                Target.Cube;
        setPipeline(target.pipeline);
    }
}
