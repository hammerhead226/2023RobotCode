// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ActiveFloor extends SubsystemBase {
  /** Creates a new ActiveFloor. */
  TalonSRX activeFMotor = new TalonSRX(Constants.ACTIVEFLOORMOTOR_PORT);

  public ActiveFloor() {}
  

  public void runConstantSpeed(){
    activeFMotor.set(ControlMode.PercentOutput, Constants.ACTIVE_FLOOR_CONSTANTSPEED);
   

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runConstantSpeed();
  }
}
