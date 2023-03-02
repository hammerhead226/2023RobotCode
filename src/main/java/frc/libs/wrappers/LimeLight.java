// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.libs.wrappers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;

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

    public static double getTargetArea() {
        return limelight.getEntry("ta").getDouble(0.0);
    }

    public static int isValidTarget() {
        long x = 0;
        return (int) (limelight.getEntry("tv").getInteger(x));
    }

}
