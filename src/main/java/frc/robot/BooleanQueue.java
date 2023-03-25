package frc.robot;

public class BooleanQueue<T extends Boolean> extends Queue<T> {
    public BooleanQueue(int capacity) {
        super(capacity);
    }

    public boolean allTrue() {
        return list.stream().allMatch(Boolean::booleanValue);
    }

    public boolean allFalse() {
        return list.stream().noneMatch(Boolean::booleanValue);
    }
}
