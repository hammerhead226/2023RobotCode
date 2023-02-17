// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.libs.wrappers.GenericMotor;
import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearSlide extends SubsystemBase {
  /** Creates a new LinearSlide. */
  private GenericMotor slider;

  private double target;
  private double speedLimit;
  private boolean manual;
  private PIDController pid;

  public LinearSlide() {
    slider = new GenericMotor(new CANSparkMax(RobotMap.SLIDER_SPARK_MAX_PORT, MotorType.kBrushless));
    slider.inverted(true);
    pid = new PIDController(Constants.LINEAR_SLIDE_GAINS[0], Constants.LINEAR_SLIDE_GAINS[1],
        Constants.LINEAR_SLIDE_GAINS[2]);
  }

  public void toggleManual() {
    manual = !manual;
  }

  public void runManual(double speed) {
    if (manual) {
      if (!(slider.getSensorPose() <= Constants.SLIDE_MIN_POSITION
          || slider.getSensorPose() >= Constants.SLIDE_MAX_POSITION))
        control(speed * Constants.LINEAR_SLIDE_COEFFICIENT);
    }
  }

  public void run() {
    if (!manual) {
      double motorSpeed = pid.calculate(slider.getSensorPose(), target);
      if (motorSpeed > speedLimit) {
        motorSpeed = speedLimit;
      } else if (motorSpeed < -speedLimit) {
        motorSpeed = -speedLimit;
      }
      control(motorSpeed);
    } else {
      control(0);
    }

  }

  public void setTarget(double t) {
    target = t;
  }

  public void setSpeedLimit(double s) {
    speedLimit = s;
  }

  public void control(double speed) {
    slider.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor Position", slider.getSensorPose());

    // This method will be called once per scheduler run
  }
}
