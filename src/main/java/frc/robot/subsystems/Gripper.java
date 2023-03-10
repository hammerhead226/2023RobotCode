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

  private boolean wristToggle = false;
  private boolean armToggle = true;
  private boolean clawToggleCone = false;
  private boolean clawToggleCube = false;

  private double armSpeedLimit;

  public Gripper() {
    wrist = new CANSparkMax(RobotMap.GRIPPER_WRIST, MotorType.kBrushless);
    claw = new GenericMotor(new TalonFX(RobotMap.CLAW_MOTOR));
    arm = new GenericMotor(new TalonFX(RobotMap.ARM_MOTOR));

    wrist.setIdleMode(IdleMode.kBrake);
    claw.setNeutralMode(PassiveMode.BRAKE);
    arm.setNeutralMode(PassiveMode.BRAKE);

    armPID = new PIDController(Constants.ARM_GAINS[0], Constants.ARM_GAINS[1], Constants.ARM_GAINS[2]);
    clawPID = new PIDController(Constants.CLAW_GAINS[0], Constants.CLAW_GAINS[1], Constants.CLAW_GAINS[2]);
    wristPID = new PIDController(Constants.WRIST_GAINS[0], Constants.WRIST_GAINS[1], Constants.WRIST_GAINS[2]);
    
    armSpeedLimit = 0.25;
  }

  public void run() {
    // CANSPARK
    if (wristToggle) {
      wrist.set(wristPID.calculate(wrist.getEncoder().getPosition(), Constants.WRIST_POS_1));
    } else {
      wrist.set(wristPID.calculate(wrist.getEncoder().getPosition(), Constants.WRIST_POS_2));
    }

    if(clawToggleCone) {
      claw.set( clawPID.calculate(claw.getSensorPose(), Constants.CLAW_CLOSE_CONE));
    } else {
      claw.set( clawPID.calculate(claw.getSensorPose(), Constants.CLAW_OPEN_CONE));
    }

    if(clawToggleCube) {
      claw.set( clawPID.calculate(claw.getSensorPose(), Constants.CLAW_CLOSE_CUBE));
    } else {
      claw.set( clawPID.calculate(claw.getSensorPose(), Constants.CLAW_OPEN_CUBE));
    }

    double armSpeed;

    if(armToggle) {
      armSpeed = armPID.calculate(arm.getSensorPose(), Constants.ARM_POS_1);
    } else {
      armSpeed = armPID.calculate(arm.getSensorPose(), Constants.ARM_POS_2);
    }

    if(armSpeed > armSpeedLimit) armSpeed = armSpeedLimit;
    else if(armSpeed < -armSpeedLimit) armSpeed = -armSpeedLimit;

    arm.set(armSpeed);

    // arm.set(Robot.m_robotContainer.test.getLeftJoyY());
    SmartDashboard.putNumber("wrist pose", wrist.getEncoder().getPosition());
    SmartDashboard.putNumber("claw pose", claw.getSensorPose());

    
    }

  public void toggleWrist() {
    wristToggle = !wristToggle;
  }

  public void toggleArm() {
    armToggle = !armToggle;
  }

  public void extendArm() {
    armToggle = false;
  }

  public void retractArm() {
    armToggle = true;
  }

  public void toggleClawCone() {
    clawToggleCone = !clawToggleCone;
  }

  public void toggleClawCube() {
    clawToggleCube = !clawToggleCube;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("periodic wrist", wrist.getEncoder().getPosition());
  }
}