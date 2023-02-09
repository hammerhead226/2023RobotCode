// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
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
  private Servo hitec;
  private Servo leftClaw;
  private Servo rightClaw;
  private PIDController pid;

  private double hitecAngle = 0.0;
  private double revLeftAngle = 0.0;
  private double revRightAngle = 180.0;

  private boolean toggleCube = false;
  private boolean toggleCone = false;
  private boolean toggleOpen = false;

  public Gripper() {
    hitec = new Servo(RobotMap.GRIPPER_HITEC);
    leftClaw = new Servo(RobotMap.REV_LEFT_HITEC);
    rightClaw = new Servo(RobotMap.REV_RIGHT_HITEC);

    pid = new PIDController(Constants.CLAW_GAINS[0], Constants.CLAW_GAINS[1], Constants.CLAW_GAINS[2]);

    hitec.setAngle(hitecAngle);
    leftClaw.setAngle(revLeftAngle);
    rightClaw.setAngle(revRightAngle);
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
   * use PID
   */
  public void incrementHitec() {
    if (hitecAngle <= 180.0) {
      hitecAngle++;
    }
    hitec.setAngle(hitecAngle);
  }

  /* implement button bindings */
  public void decrementHitec() {
    if (hitecAngle >= 180.0) {
      hitecAngle--;
    }
    hitec.setAngle(hitecAngle);
  }

  public void setClawToCube() {
    leftClaw.setAngle(pid.calculate(leftClaw.get(), Constants.LEFT_CLAW_CUBE)); // getAngle is a ballsack function
    rightClaw.setAngle(pid.calculate(rightClaw.get(), Constants.RIGHT_CLAW_CUBE));
  }

  public void setClawToCone() {
    leftClaw.setAngle(pid.calculate(leftClaw.get(), Constants.LEFT_CLAW_CONE));
    rightClaw.setAngle(pid.calculate(rightClaw.get(), Constants.RIGHT_CLAW_CONE));
  }

  public void close() {
    leftClaw.setAngle(pid.calculate(leftClaw.get(), Constants.LEFT_CLAW_CLOSE));
    rightClaw.setAngle(pid.calculate(rightClaw.get(), Constants.RIGHT_CLAW_CLOSE));
  }

  public void open() {
    leftClaw.setAngle(pid.calculate(leftClaw.get(), Constants.LEFT_CLAW_RELEASE));
    rightClaw.setAngle(pid.calculate(rightClaw.get(), Constants.RIGHT_CLAW_RELEASE));
  }

  // don't think we need this method but why not
  public void set() {
    hitec.setAngle(hitecAngle);
    leftClaw.setAngle(revLeftAngle);
    rightClaw.setAngle(revRightAngle);
  }

  public void run() {
    if (toggleCube) {
      setClawToCube();
    } else {
      close();
    }

    if (toggleCone) {
      setClawToCone();
    } else {
      close();
    }

    if (toggleOpen) {
      open();
    } else {
      close();
    }

  }

  public void toggleCube() {
    toggleCube = !toggleCube;
  }

  public void toggleCone() {
    toggleCone = !toggleCone;
  }

  public void toggleOpen() {
    toggleOpen = !toggleOpen;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
