package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class Limelight {
    private final NetworkTableEntry horizontalOffsetEntry;
    private final NetworkTableEntry verticalOffsetEntry;
    private final NetworkTableEntry visibleEntry;
    private final NetworkTableEntry pipelineEntry;

    public Limelight(String instance) {
        var networkTable = NetworkTableInstance.getDefault().getTable(instance);
        horizontalOffsetEntry = networkTable.getEntry("tx");
        verticalOffsetEntry = networkTable.getEntry("ty");
        visibleEntry = networkTable.getEntry("tv");
        pipelineEntry = networkTable.getEntry("pipeline");
    }

    public double getHorizontalOffset() {
        return horizontalOffsetEntry.getValue().getDouble();
    }

    public double getVerticalOffset() {
        return verticalOffsetEntry.getValue().getDouble();
    }

    public boolean getVisible() {
        return visibleEntry.getValue().getBoolean();
    }

    public void setPipeline(int pipeline) {
        pipelineEntry.setNumber(pipeline);
    }

    public double getPipeline() {
        return pipelineEntry.getValue().getDouble();
    }
}