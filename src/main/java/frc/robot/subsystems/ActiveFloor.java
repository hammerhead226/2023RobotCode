// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.wrappers.GenericMotor;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ActiveFloor extends SubsystemBase {
  /** Creates a new ActiveFloor. */
  GenericMotor activeMotor;

  public ActiveFloor() {
    TalonFX fal = new TalonFX(RobotMap.ACTIVE_FLOOR_MOTOR_PORT, Constants.CANBUS);
    fal.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100);
    activeMotor = new GenericMotor(fal);
    
  }

  public void runConstantSpeedInward() {
    activeMotor.set(Constants.ACTIVE_FLOOR_CONSTANT_SPEED);
  }

  public void runConstantSpeedOutward() {
    activeMotor.set(-Constants.ACTIVE_FLOOR_CONSTANT_SPEED);
  }

  public void stop() {
    activeMotor.set(0);
  }

  public void control(double speed) {
    activeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
