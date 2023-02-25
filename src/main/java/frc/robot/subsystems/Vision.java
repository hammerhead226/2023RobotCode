// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.wrappers.SharkSight;

public class Vision extends SubsystemBase {

  private static boolean toggleEnabled = SharkSight.isEnabled();


  public Vision() {}

  public static void toggle() {
    if (toggleEnabled) {
      SharkSight.disable();
      toggleEnabled = false;
    } else {
      SharkSight.enable();
      toggleEnabled = true;
    }
  }

  public static void shutdown() {
    SharkSight.shutdown();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SharkSight.updateIntakeClosest();
    SharkSight.updateGripperClosest();
  }
}
