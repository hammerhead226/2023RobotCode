// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.wrappers.Jetson;

public class Vision extends SubsystemBase {

  private static boolean toggleEnabled = true;


  public Vision() {}

  public static void toggle() {
    if (toggleEnabled) {
      Jetson.disable();
      toggleEnabled = false;
    } else {
      Jetson.enable();
      toggleEnabled = true;
    }
  }

  public static void shutdown() {
    Jetson.shutdown();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
