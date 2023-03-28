// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.wrappers.GenericMotor;
import frc.libs.wrappers.GenericMotor.PassiveMode;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Gripper extends SubsystemBase {
  /**
   * Three servos in total
   * 1 NEO motor as the arm
   * 1 NEO motor to open/close the gripper
   * 1 servo to rotate hand
   * up and down position for arm (toggle) --> A button
   * pincing (toggle) --> B button
   * wrist rotation (180 or 0 degrees; also toggle) --> X button
   */

  // private TalonFX wrist;
  private CANSparkMax wrist;
  private GenericMotor arm;
  private GenericMotor claw; 

  private PIDController armPID;
  private PIDController clawPID;
  private PIDController wristPID;

  private boolean wristToggle = true;
  private boolean armToggle = true;

  private double armTarget = 0;

  private boolean clawToggle = true;

  private boolean cubeMode = false;

  private double armSpeedLimit;

  public Gripper() {
    // wrist = new CANSparkMax(RobotMap.GRIPPER_WRIST, MotorType.kBrushless);
    claw = new GenericMotor(new TalonFX(RobotMap.CLAW_MOTOR, Constants.CANBUS));
    arm = new GenericMotor(new TalonFX(RobotMap.ARM_MOTOR, Constants.CANBUS));

    // wrist.setIdleMode(IdleMode.kBrake);
    claw.setNeutralMode(PassiveMode.BRAKE);
    arm.setNeutralMode(PassiveMode.BRAKE);

    armPID = new PIDController(Constants.ARM_GAINS[0], Constants.ARM_GAINS[1], Constants.ARM_GAINS[2]);
    clawPID = new PIDController(Constants.CLAW_GAINS[0], Constants.CLAW_GAINS[1], Constants.CLAW_GAINS[2]);
    wristPID = new PIDController(Constants.WRIST_GAINS[0], Constants.WRIST_GAINS[1], Constants.WRIST_GAINS[2]);
    
    armSpeedLimit = 0.8;
  }

  public void run() {

    // if(armToggle) wristToggle = false;

    // if(armTarget != 0)
    //   wristPID.setP(0.23);
    // else 
    //   wristPID.setP(Constants.WRIST_GAINS[0]);

    // SmartDashboard.putNumber("wrist p", wristPID.getP());
    // SmartDashboard.putNumber("wrist pose", wrist.getEncoder().getPosition());
    // // CANSPARK
    // if (wristToggle) {
    //   double wristPose;
    //   if(armToggle) wristPose = Constants.PERFECT_WRIST_POS_1;
    //   else wristPose = Constants.ADJUSTED_WRIST_POS_1;
    //   wrist.set(wristPID.calculate(wrist.getEncoder().getPosition(), wristPose));
    // } else {
    //   double wristPose;
    //   if(armToggle) wristPose = Constants.PERFECT_WRIST_POS_2;
    //   else wristPose = Constants.ADJUSTED_WRIST_POS_2;
    //   wrist.set(wristPID.calculate(wrist.getEncoder().getPosition(), wristPose));
    // }

    if(clawToggle) {
      if(cubeMode) {
        claw.set( clawPID.calculate(claw.getSensorPose(), Constants.CLAW_CLOSE_CUBE));
      }
      else {
        claw.set( clawPID.calculate(claw.getSensorPose(), Constants.CLAW_CLOSE_CONE));
      }
    }
    else {
      claw.set( clawPID.calculate(claw.getSensorPose(), Constants.CLAW_OPEN_CONE));
    }

    double armSpeed = armPID.calculate(arm.getSensorPose(), armTarget);

    if(armSpeed > armSpeedLimit) armSpeed = armSpeedLimit;
    else if(armSpeed < -armSpeedLimit) armSpeed = -armSpeedLimit;

    arm.set(armSpeed);

    // arm.set(Robot.m_robotContainer.manip.getLeftJoyY());
    SmartDashboard.putBoolean("claw toggle", clawToggle);
    SmartDashboard.putBoolean("cube mode", cubeMode);

    
    }

  public void toggleWrist() {
    wristToggle = !wristToggle;
  }

  public void toggleArm() {
    armToggle = !armToggle;
  }

  public void extendArm() {
    armTarget = Constants.ARM_POS_2;
  }

  public void retractArm() {
    armTarget = Constants.ARM_POS_1;
  }

  public void armHoldPosition() {
    if(cubeMode) armTarget = 0;
    else armTarget = 3000;
  }

  public void setArmTarget(double target) {
    armTarget = target;
  }

  public void toggleCubeMode() {
    cubeMode = !cubeMode;
  }

  public void toggleClaw() {
    clawToggle = !clawToggle;
  }

  public void openClaw() {
    clawToggle = false;
  }
  
  public void closeClaw() {
    clawToggle = true;
  }

  public void wristFalconUp() {
    wristToggle = false;
  }

  public void wristFalconDown() {
    wristToggle = true;
  }

  public boolean getCubeMode() {
    return cubeMode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("periodic wrist", wrist.getEncoder().getPosition());
    // SmartDashboard.putNumber("wrist pose", wrist.getEncoder().getPosition());
    
  }
}