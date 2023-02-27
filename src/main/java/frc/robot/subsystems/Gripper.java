// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
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
   * up and down position (toggle) --> A button
   * pincing (toggle) --> B button
   * wrist rotation (180 or 0 degrees; also toggle) --> X button
   */
  private Servo wrist;
  private CANSparkMax joint;
  private CANSparkMax claw; 

  private PIDController jointPID;
  private PIDController clawPID;

  private RelativeEncoder jointEncoder;
  private RelativeEncoder clawEncoder;

  private boolean isGripped = false;
  private boolean jointToggle = false;
  private boolean clawToggle = false;

  public Gripper() {
    wrist = new Servo(RobotMap.GRIPPER_HITEC);
    joint = new CANSparkMax(RobotMap.GRIPPER_MOTORS[0], MotorType.kBrushless);
    claw = new CANSparkMax(RobotMap.GRIPPER_MOTORS[1], MotorType.kBrushless);

    jointEncoder = joint.getEncoder();
    clawEncoder = claw.getEncoder();

    jointPID = new PIDController(Constants.JOINT_GAINS[0], Constants.JOINT_GAINS[1], Constants.JOINT_GAINS[2]);
    clawPID = new PIDController(Constants.CLAW_GAINS[0], Constants.CLAW_GAINS[1], Constants.CLAW_GAINS[2]);

    wrist.setAngle(Constants.ARM_PIVOT_ANGLE);
  }

  public void run() {
    if(clawToggle) {
      control(clawPID.calculate(claw.get(), Constants.CLAW_CLOSE), Constants.CLAW_SETTING);
    } else {
      control(clawPID.calculate(claw.get(), Constants.CLAW_OPEN), Constants.CLAW_SETTING);
    }

    if(jointToggle) {
      control(jointPID.calculate(joint.get(), Constants.JOINT_POS_1), Constants.JOINT_SETTING);
    } else {
      control(jointPID.calculate(joint.get(), Constants.JOINT_POS_2), Constants.JOINT_SETTING);
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
