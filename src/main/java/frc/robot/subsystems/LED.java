// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class LED extends SubsystemBase {
  double sparkSpeed = Constants.STARTING_LED;
  Spark spark;
  
  /** Creates a new LED. */
  public LED() {
    spark = new Spark(RobotMap.SPARK_MOTOR);
  }

  public void ledOff(){
    spark.disable();
  }

  public void ledYellow(){
    sparkSpeed = Constants.LED_YELLOW;
    
  }

  public void ledViolet(){
    sparkSpeed = Constants.LED_VIOLET;
    
  }

  public void incrementLED(){
    sparkSpeed += 0.01;
  }

  public void decrementLED(){
    sparkSpeed -= 0.01;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    spark.set(sparkSpeed);
  }
}
