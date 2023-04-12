// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearSlide extends SubsystemBase {
  /** Creates a new LinearSlide. */
  private CANSparkMax slider;

  private double target;
  private double extendSpeedLimit;
  private double retractSpeedLimit;
  private boolean manual;
  private PIDController pid;

  public LinearSlide() {
    slider = new CANSparkMax(RobotMap.SLIDER_SPARK_MAX_PORT, MotorType.kBrushless);
    slider.setInverted(Constants.LS_SET_INVERTED);

    
    slider.setInverted(true);

    pid = new PIDController(Constants.LINEAR_SLIDE_GAINS[0], Constants.LINEAR_SLIDE_GAINS[1],
        Constants.LINEAR_SLIDE_GAINS[2]);

    extendSpeedLimit = 0.8;
    retractSpeedLimit = 0.2;
    manual = false;
  }

  public void toggleManual() {
    manual = !manual;
  }

  int sustain = 0;
  public void run() {
    if (!manual) {
      double motorSpeed = pid.calculate(slider.getEncoder().getPosition(), target);
      if(target == 0) {
        if (motorSpeed > retractSpeedLimit) {
          motorSpeed = retractSpeedLimit;
        } else if (motorSpeed < -retractSpeedLimit) {
          motorSpeed = -retractSpeedLimit;
        }
      }
      else {
        if (motorSpeed > extendSpeedLimit) {
          motorSpeed = extendSpeedLimit;
        } else if (motorSpeed < -extendSpeedLimit) {
          motorSpeed = -extendSpeedLimit;
        }
      }
      
      // SmartDashboard.putNumber("motor speed", motorSpeed);
      // control(Robot.m_robotContainer.manip.getLeftJoyY());
      control(motorSpeed);
    }
    SmartDashboard.putNumber("neo pose", slider.getEncoder().getPosition());
  }

  public void setTarget(double t) {
    target = t;
  }

  public void control(double speed) {
    slider.set(speed);
  }

  @Override
  public void periodic() {}
}
