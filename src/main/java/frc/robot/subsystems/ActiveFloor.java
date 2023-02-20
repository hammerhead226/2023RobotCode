// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ActiveFloor extends SubsystemBase {
  /** Creates a new ActiveFloor. */
  TalonFX activeMotor;

  public ActiveFloor() {
    activeMotor = new TalonFX(RobotMap.ACTIVE_FLOOR_MOTOR_PORT);
  }

  public void runConstantSpeed() {
    activeMotor.set(ControlMode.PercentOutput, Constants.ACTIVE_FLOOR_CONSTANT_SPEED);
  }

  public void stop() {
    activeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void control(double speed) {
    activeMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
