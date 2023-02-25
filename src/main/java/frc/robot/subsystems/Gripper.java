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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

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

  private static NetworkTable jetson = NetworkTableInstance.getDefault().getTable("SharkSight");

  public Gripper() {
    wrist = new Servo(0);
    joint = new GenericMotor(new CANSparkMax(0, MotorType.kBrushless));
    claw = new GenericMotor(new CANSparkMax(0, MotorType.kBrushless));

    joint.setSensorPose(0);
    claw.setSensorPose(0);

    jointPID = new PIDController(0, 0, 0);
    clawPID = new PIDController(0, 0, 0);

    wrist.setAngle(0);
  }

  public void run() {
    if(clawToggle) {
      control(clawPID.calculate(claw.getSensorPose(), 0), 0);
    } else {
      control(clawPID.calculate(claw.getSensorPose(), 0), 0);
    }

    if(jointToggle) {
      control(jointPID.calculate(joint.getSensorPose(), 0), 0);
    } else {
      control(jointPID.calculate(joint.getSensorPose(), 0), 0);
    }

    if(isGripped) {
      control(0, 0);
    } else {
      control(0, 0);
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

  public void positionWrist() {
    int reverse = 0;
    for (int i = 0; i < 10; i++) {
      if (jetson.getEntry("Cone Rotation").getString("").equals("Upside Down")) {
        reverse++;
      }
    }

    if (reverse > 5) {
      wrist.set(180);
    }
    else {
      wrist.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}