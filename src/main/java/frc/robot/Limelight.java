package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class Limelight {
    private final NetworkTableEntry validEntry; // 0 or 1
    private final NetworkTableEntry horizontalOffsetEntry; // -27 degrees to 27 degrees
    private final NetworkTableEntry verticalOffsetEntry; // -20.5 degrees to 20.5 degrees
    private final NetworkTableEntry areaEntry; // 0% to 100% of image

    public Limelight(String name) {
        var table = NetworkTableInstance.getDefault().getTable(name);
        validEntry = table.getEntry("tv");
        horizontalOffsetEntry = table.getEntry("tx");
        verticalOffsetEntry = table.getEntry("ty");
        areaEntry = table.getEntry("ta");

    }

    public boolean getValid() {
        return getDouble(validEntry) == 1d;
    }

    public double getHorizontalOffset() {
        return getDouble(horizontalOffsetEntry);
    }

    public double getVerticalOffset() {
        return getDouble(verticalOffsetEntry);
    }

    public double getArea() {
        return getDouble(areaEntry);
    }

    public void setPipeline(Alliance alliance) {
        //int id = alliance == Alliance.Blue ? 0 : 1;
        //networkTable.getEntry("pipeline").setNumber(id);
    }

    private static double getDouble(NetworkTableEntry entry) {
        return entry.getValue().getDouble();
    }
}