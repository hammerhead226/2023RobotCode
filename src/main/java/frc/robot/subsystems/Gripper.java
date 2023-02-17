// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Gripper extends SubsystemBase {
  /**
   * Three servos in total
   * 1 HiTEC servo to rotate linear slide
   * 2 REV Smart Robot Servos that move the claw
   */
  private Servo armPivot;
  private Servo leftClaw;
  private Servo rightClaw;

  private boolean isGripped = false;

  public Gripper() {
    armPivot = new Servo(RobotMap.GRIPPER_HITEC);
    leftClaw = new Servo(RobotMap.REV_LEFT_HITEC);
    rightClaw = new Servo(RobotMap.REV_RIGHT_HITEC);

    armPivot.setAngle(Constants.ARM_PIVOT_ANGLE);
    leftClaw.setAngle(Constants.REV_LEFT_ANGLE);
    rightClaw.setAngle(Constants.REV_RIGHT_ANGLE);
  }

  /**
   * the servo has to start out at 0 degrees
   * whenever a button/trigger is held, the servo should turn without any jerk
   * if we incremement angle by 1 degree, it should theoretically move smoothly
   * should be a separate method to increment/decrement angles
   * one method to increment angles
   * one method to decrement angles
   * do this for the Hitec and the claws
   * can have one method to just set all the servos tbh
   * claws can be set to different modes
   * instead of manually controlling claw movement, set it to 4 different modes
   * cube, cone, close, and release mode
   */

  /* implement button bindings */

  public void close() {
    leftClaw.setAngle(Constants.LEFT_CLAW_CLOSE);
    rightClaw.setAngle(Constants.RIGHT_CLAW_CLOSE);
  }

  public void open() {
    leftClaw.setAngle(Constants.LEFT_CLAW_CLOSE);
    rightClaw.setAngle(Constants.RIGHT_CLAW_CLOSE);
  }

  public void run() {
    if (isGripped) {
      close();

    } else {
      open();

    }
  }

  public void control(double armPivotAngle, double leftClawAngle, double rightClawAngle) {
    armPivot.setAngle(armPivotAngle);
    leftClaw.setAngle(leftClawAngle);
    rightClaw.setAngle(rightClawAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
