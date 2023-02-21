// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.wrappers.GenericMotor;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Gripper extends SubsystemBase {
  /**
   * Three servos in total
   * 1 NEO motor as the joint
   * 1 NEO motor to open/close the gripper
   * 1 servo to rotate hand
   */
  private Servo wrist;
  private GenericMotor joint;
  private GenericMotor claw; 

  private PIDController jointPID;
  private PIDController clawPID;

  private boolean isGripped = false;
  private boolean jointToggle = false;
  private boolean clawToggle = false;

  public Gripper() {
    wrist = new Servo(RobotMap.GRIPPER_HITEC);
    joint = new GenericMotor(new CANSparkMax(RobotMap.JOINT, MotorType.kBrushless));
    claw = new GenericMotor(new CANSparkMax(RobotMap.CLAW, MotorType.kBrushless));

    joint.setSensorPose(0);
    claw.setSensorPose(0);

    jointPID = new PIDController(Constants.JOINT_GAINS[0], Constants.JOINT_GAINS[1], Constants.JOINT_GAINS[2]);
    clawPID = new PIDController(Constants.CLAW_GAINS[0], Constants.CLAW_GAINS[1], Constants.CLAW_GAINS[2]);

    wrist.setAngle(Constants.ARM_PIVOT_ANGLE);
  }

  public void run() {
    if(clawToggle) {
      control(clawPID.calculate(claw.getSensorPose(), Constants.CLAW_CLOSE), Constants.CLAW_SETTING);
    } else {
      control(clawPID.calculate(claw.getSensorPose(), Constants.CLAW_OPEN), Constants.CLAW_SETTING);
    }

    if(jointToggle) {
      control(jointPID.calculate(joint.getSensorPose(), Constants.JOINT_POS_1), Constants.JOINT_SETTING);
    } else {
      control(jointPID.calculate(joint.getSensorPose(), Constants.JOINT_POS_2), Constants.JOINT_SETTING);
    }

    if(isGripped) {
      control(Constants.WRIST_POS_1, Constants.WRIST_SETTING);
    } else {
      control(Constants.WRIST_POS_2, Constants.WRIST_SETTING);
    }
  }

  public void toggleWrist() {
    isGripped = !isGripped;
  }

  public void toggleJoint() {
    jointToggle = !jointToggle;
  }

  public void toggleClaw() {
    clawToggle = !clawToggle;
  }

  public void control(double position, int setting) {
    if(setting == 0) {
      wrist.setAngle(position);
    } else if (setting == 1) {
      joint.set(position);
    } else if (setting == 2) {
      claw.set(position);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
