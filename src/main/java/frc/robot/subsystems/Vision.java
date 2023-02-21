// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.wrappers.Jetson;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase {
  private static final Jetson vision = new Jetson();
  private static NetworkTable jetson = NetworkTableInstance.getDefault().getTable("Jetson");

  public static boolean toggleEnabled = true;

  public static void toggle() {
    if (toggleEnabled) {
      disable();
      toggleEnabled = false;
    } else {
      enable();
      toggleEnabled = true;
    }
  }
  public static void enable() {
    jetson.getEntry("Enabled").setBoolean(true);
  }

  public static void disable() {
    jetson.getEntry("Disabled").setBoolean(false);
  }

  public static void shutdown() {
    jetson.getEntry("Shutdown").setBoolean(true);
  }

  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
