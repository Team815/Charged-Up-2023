package frc.robot;

import edu.wpi.first.math.Pair;

import java.util.LinkedList;

public class DoubleQueue {
    private final LinkedList<Double> list = new LinkedList<>();
    private final int capacity;

    public DoubleQueue(int capacity) {
        this.capacity = capacity;
    }

    public void add(double val) {
        list.add(val);
        while (list.size() > capacity) {
            list.pop();
        }
    }

    public boolean isFull() {
        return list.size() == capacity;
    }

    public double largest() {
        return list.stream().max(Double::compare).orElse(0d);
    }

    public double smallest() {
        return list.stream().min(Double::compare).orElse(0d);
    }

    public Pair<Double, Double> range() {
        return new Pair<>(smallest(), largest());
    }
}
