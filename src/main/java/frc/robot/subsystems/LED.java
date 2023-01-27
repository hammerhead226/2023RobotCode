// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {

  Spark spark;
  /** Creates a new LED. */
  public LED() {
    spark = new Spark(Constants.SPARK_MOTOR);
  }

  public void ledOff(){
    spark.set(Constants.LED_OFF);
  }

  public void ledYellow(){
    spark.set(Constants.LED_YELLOW);
  }

  public void ledVoilet(){
    spark.set(Constants.LED_VIOLET);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ledYellow();
  }
}
