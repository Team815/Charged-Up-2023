package frc.robot;

import java.util.LinkedList;

public class Queue<T> {
    protected final LinkedList<T> list = new LinkedList<>();
    private final int capacity;

    public Queue(int capacity) {
        this.capacity = capacity;
    }

    public void add(T val) {
        while (list.size() >= capacity) {
            list.pop();
        }
        list.add(val);
    }

    public boolean isFull() {
        return list.size() == capacity;
    }
}
