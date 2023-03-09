// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.libs.wrappers.GenericMotor;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearSlide extends SubsystemBase {
  /** Creates a new LinearSlide. */
  private CANSparkMax slider;

  private double target;
  private double speedLimit;
  private boolean manual;
  private PIDController pid;

  public LinearSlide() {
    slider = new CANSparkMax(RobotMap.SLIDER_SPARK_MAX_PORT, MotorType.kBrushless);
    slider.setInverted(Constants.LS_SET_INVERTED);

    pid = new PIDController(Constants.LINEAR_SLIDE_GAINS[0], Constants.LINEAR_SLIDE_GAINS[1],
        Constants.LINEAR_SLIDE_GAINS[2]);

    speedLimit = 0.5;
    manual = false;
  }

  public void toggleManual() {
    manual = !manual;
  }

  public void run() {
    if (!manual && !(RobotContainer.elevator.get() >= Constants.SLIDE_DISABLE_POSE)) {
      double motorSpeed = pid.calculate(slider.getEncoder().getPosition(), target);

      if (motorSpeed > speedLimit) {
        motorSpeed = speedLimit;
      } else if (motorSpeed < -speedLimit) {
        motorSpeed = -speedLimit;
      }

      control(motorSpeed);
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
  public void periodic() {}
}
