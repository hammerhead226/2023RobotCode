// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.libs.wrappers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * @author Anish Chandra
 *         Simplifies Data Collection from LimeLight
 */
public class LimeLight {
    private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public static double getHorizontalOffset() {
        return limelight.getEntry("tx").getDouble(0.0);
    }

    public static double getVerticalOffset() {
        return limelight.getEntry("ty").getDouble(0.0);
    }

    public double getTargetArea() {
        return limelight.getEntry("ta").getDouble(0.0);
    }

    public double getValue() {
        return limelight.getEntry("tv").getDouble(0.0);
    }

    public double getShortSide() {
        return limelight.getEntry("tshort").getDouble(0.0);
    }

    public double getLongSide() {
        return limelight.getEntry("tlong").getDouble(0.0);
    }

    public double getHorizontalSide() {
        return limelight.getEntry("thor").getDouble(0.0);
    }

    public double getVerticalSide() {
        return limelight.getEntry("tvert").getDouble(0.0);
    }


}
