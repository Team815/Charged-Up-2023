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

    public static final Limelight limelightField = new Limelight("limelight-field");

    private final NetworkTable networkTable;

    private Limelight(String instance) {
        networkTable = NetworkTableInstance.getDefault().getTable(instance);
    }

    public double getX() {
        return networkTable.getEntry("tx").getDouble(0);
    }

    public double getY() {
        return networkTable.getEntry("ty").getDouble(0);
    }

    public boolean getVisible() {
        return networkTable.getEntry("tv").getBoolean(false);
    }

    public void setPipeline(Alliance alliance) {
        //int id = alliance == Alliance.Blue ? 0 : 1;
        //networkTable.getEntry("pipeline").setNumber(id);
    }
}