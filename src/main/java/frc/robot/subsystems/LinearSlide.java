// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.libs.wrappers.LoggedTunableNumber;
import frc.libs.wrappers.GenericMotor.PassiveMode;
import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearSlide extends SubsystemBase {
  /** Creates a new LinearSlide. */

  private static LoggedTunableNumber highLsKp = new LoggedTunableNumber("LS_High/kP");
  private static LoggedTunableNumber highLsKi = new LoggedTunableNumber("LS_High/Ki");
  private static LoggedTunableNumber highLsKd = new LoggedTunableNumber("LS_High/Kd");

  private static LoggedTunableNumber lowLsKp = new LoggedTunableNumber("LS_Low/kP");
  private static LoggedTunableNumber lowLsKi = new LoggedTunableNumber("LS_Low/Ki");
  private static LoggedTunableNumber lowLsKd = new LoggedTunableNumber("LS_Low/Kd");

  private static LoggedTunableNumber lsHigh = new LoggedTunableNumber("LS/High");
  private static LoggedTunableNumber lsMid = new LoggedTunableNumber("LS/Mid");
  private static LoggedTunableNumber lsRetracted = new LoggedTunableNumber("LS/Retract");
  private static LoggedTunableNumber lsSub = new LoggedTunableNumber("LS/Sub");

  static {
    highLsKp.initDefault(0);
    highLsKi.initDefault(0);
    highLsKd.initDefault(0);

    lowLsKp.initDefault(0);
    lowLsKi.initDefault(0);
    lowLsKd.initDefault(0);

    lsHigh.initDefault(0);
    lsMid.initDefault(0);
    lsRetracted.initDefault(0);
    lsSub.initDefault(0);
  }

  private static double elsHigh;
  private static double elsMid;
  private static double elsRetracted;
  private static double elsSub;

  private static double[] pHigh;
  private static double[] pLow;

  private TalonFX slider;

  private double target;
  private double extendSpeedLimit;
  private double retractSpeedLimit;
  private boolean manual;
  private PIDController pid;

  public LinearSlide() {
    slider = new TalonFX(RobotMap.SLIDER_PORT,"CAN Bus 2");
    slider.setInverted(Constants.LS_SET_INVERTED);

    slider.configOpenloopRamp(0.6);
    
    slider.setNeutralMode(NeutralMode.Brake);
    slider.configNeutralDeadband(0.1);
    // slider.setInverted(true);

    pid = new PIDController(Constants.LINEAR_SLIDE_GAINS_HIGH[0], Constants.LINEAR_SLIDE_GAINS_HIGH[1],
        Constants.LINEAR_SLIDE_GAINS_HIGH[2]);
    
    pHigh[0] = highLsKp.get();
    pHigh[1] = highLsKi.get();
    pHigh[2] = highLsKd.get();

    pLow[0] = lowLsKp.get();
    pLow[1] = lowLsKi.get();
    pLow[2] = lowLsKd.get();

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
      pid.setPID(pHigh[0], pHigh[1], pHigh[2]);
      if (err <= 20000) {
        pid.setPID(pHigh[0], pHigh[1], pHigh[2]);
      } else {
        pid.setPID(pLow[0], pLow[1], pLow[2]);
      }
      
      double motorSpeed = pid.calculate(slider.getSelectedSensorPosition(), target);
      if(target == elsRetracted) {
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

  public static double getHigh() {
    return lsHigh.get();
  }

  public static double getMid() {
    return lsMid.get();
  }

  public static double getRetract() {
    return lsRetracted.get();
  }

  public static double getSub() {
    return lsSub.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("slide deez nuts into yo", getPosition());

    if (highLsKp.hasChanged(hashCode()) || highLsKi.hasChanged(hashCode()) || highLsKd.hasChanged(hashCode()) || 
        lowLsKp.hasChanged(hashCode()) || lowLsKi.hasChanged(hashCode()) || lowLsKd.hasChanged(hashCode()) ||
        lsHigh.hasChanged(hashCode()) || lsMid.hasChanged(hashCode()) || lsRetracted.hasChanged(hashCode()) || lsSub.hasChanged(hashCode())) {

      pHigh[0] = highLsKp.get();
      pHigh[1] = highLsKi.get();
      pHigh[2] = highLsKd.get();
  
      pLow[0] = lowLsKp.get();
      pLow[1] = lowLsKi.get();
      pLow[2] = lowLsKd.get();

      elsHigh = lsHigh.get();
      elsMid = lsMid.get();
      elsRetracted = lsRetracted.get();
      elsSub = lsSub.get();
    }
  }
}