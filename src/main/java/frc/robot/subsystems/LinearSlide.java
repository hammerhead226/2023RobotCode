// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

// -60000 high
// -29966 mid
// -8180
import frc.libs.wrappers.GenericMotor.PassiveMode;
import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearSlide extends SubsystemBase {
  /** Creates a new LinearSlide. */
  private TalonFX slider;

  private double target;
  private double extendSpeedLimit;
  private double retractSpeedLimit;
  private boolean manual;
  private PIDController pid;

  public LinearSlide() {
    slider = new TalonFX(RobotMap.SLIDER_PORT, Constants.CANBUS);
    slider.setInverted(Constants.LS_SET_INVERTED);

    slider.configOpenloopRamp(0.6);
    
    slider.setNeutralMode(NeutralMode.Brake);
    slider.configNeutralDeadband(0.1);
    // slider.setInverted(true);

    pid = new PIDController(Constants.LINEAR_SLIDE_GAINS_HIGH[0], Constants.LINEAR_SLIDE_GAINS_HIGH[1],
        Constants.LINEAR_SLIDE_GAINS_HIGH[2]);

    extendSpeedLimit = 0.80;
    retractSpeedLimit = 0.65;
    manual = false;
  }

  public void toggleManual() {
    manual = !manual;
  }

  int sustain = 0;
  public void run() {
    if (!manual) {
      double err = Math.abs(target - getPosition());
      pid.setPID(Constants.LINEAR_SLIDE_GAINS_HIGH[0], Constants.LINEAR_SLIDE_GAINS_HIGH[1], Constants.LINEAR_SLIDE_GAINS_HIGH[2]);
      if (err <= 31000) {
        pid.setPID(Constants.LINEAR_SLIDE_GAINS_HIGH[0], Constants.LINEAR_SLIDE_GAINS_HIGH[1], Constants.LINEAR_SLIDE_GAINS_HIGH[2]);
      } else {
        pid.setPID(Constants.LINEAR_SLIDE_GAINS_LOW[0], Constants.LINEAR_SLIDE_GAINS_LOW[1], Constants.LINEAR_SLIDE_GAINS_LOW[2]);
      }
      
      double motorSpeed = pid.calculate(slider.getSelectedSensorPosition(), target);
      if(target == Constants.LS_RETRACTED) {
        if (motorSpeed > retractSpeedLimit) {
          motorSpeed = retractSpeedLimit;
        } else if (motorSpeed < -retractSpeedLimit) {
          motorSpeed = -retractSpeedLimit;
        }

        // if (Math.abs(slider.getSelectedSensorPosition()) <= 0.8 * Math.abs(target)) {
        //   motorSpeed = 0;
        // }
      }
      else {
        if (motorSpeed > extendSpeedLimit) {
          motorSpeed = extendSpeedLimit;
        } else if (motorSpeed < -extendSpeedLimit) {
          motorSpeed = -extendSpeedLimit;
        }

        // if (Math.abs(slider.getSelectedSensorPosition()) <= 0.8 * Math.abs(target)) {
        //   motorSpeed = 0;
        // }
      }

      if (Math.abs(err) <= 100) {
        motorSpeed = 0;
      }
      
      // SmartDashboard.putNumber("motor speed", motorSpeed);
      // control(Robot.m_robotContainer.manip.getLeftJoyY());
      // motorSpeed = Math.abs(motorSpeed) < 0.09 ? 0 : motorSpeed;
      control(motorSpeed);
    }
    SmartDashboard.putNumber("linear slide extension", slider.getSelectedSensorPosition());
  }

  public void setTarget(double t) {
    target = t;
  }

  public double getTarget() {
    return target;
  }

  public void control(double speed) {
    slider.set(ControlMode.PercentOutput, speed);
  }

  public double getPosition() {
    return slider.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("slide deez nuts into yo", getPosition());
    
  }
}