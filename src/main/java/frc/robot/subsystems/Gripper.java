// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.wrappers.GenericMotor;
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
  private Servo wrist;
  private TalonFX arm;
  private TalonFX claw; 

  private PIDController armPID;
  private PIDController clawPID;

  private boolean isGripped = false;
  private boolean armToggle = false;
  private boolean clawToggle = false;

  public Gripper() {
    wrist = new Servo(RobotMap.GRIPPER_HITEC);
    claw = new TalonFX(RobotMap.CLAW_MOTOR);
    arm = new TalonFX(RobotMap.ARM_MOTOR);
    claw.setNeutralMode(NeutralMode.Brake);
    arm.setNeutralMode(NeutralMode.Brake);

    armPID = new PIDController(Constants.ARM_GAINS[0], Constants.ARM_GAINS[1], Constants.ARM_GAINS[2]);
    clawPID = new PIDController(Constants.CLAW_GAINS[0], Constants.CLAW_GAINS[1], Constants.CLAW_GAINS[2]);

    wrist.setAngle(Constants.WRIST_POS_2);
  }

  public void run() {
    if(clawToggle) {
      claw.set(ControlMode.PercentOutput, clawPID.calculate(claw.getSelectedSensorPosition(), Constants.CLAW_CLOSE));
    } else {
      claw.set(ControlMode.PercentOutput, clawPID.calculate(claw.getSelectedSensorPosition(), Constants.CLAW_OPEN));
    }


    if(armToggle) {
      arm.set(ControlMode.PercentOutput, armPID.calculate(arm.getSelectedSensorPosition(), Constants.ARM_POS_1));
    } else {
      arm.set(ControlMode.PercentOutput, armPID.calculate(arm.getSelectedSensorPosition(), Constants.ARM_POS_2));
    }

    if(isGripped) {
      wrist.setAngle(Constants.WRIST_POS_1);
    } else {
      wrist.setAngle(Constants.WRIST_POS_2);
    }

    SmartDashboard.putNumber("claw Position: ", claw.getSelectedSensorPosition());
    SmartDashboard.putNumber("arm pose", arm.getSelectedSensorPosition());
  }

  public void toggleWrist() {
    isGripped = !isGripped;
  }

  public void toggleArm() {
    armToggle = !armToggle;
  }

  public void toggleClaw() {
    clawToggle = !clawToggle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
