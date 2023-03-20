package frc.robot;

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
