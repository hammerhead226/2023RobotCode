// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ScoringStateManager extends SubsystemBase {
  public LinearSlide linearSlide = new LinearSlide();
  public Gripper gripper = new Gripper();
  public Elevator elevator = new Elevator();
  public Intake intake = new Intake();

  public ScoringStateManager() {}

  public double getIntakeState() {
    return intake.getIntake();
  }

  public double getLinearSlideState() {
    return linearSlide.getPosition();
  }
  
  public double getElevatorState() {
    return elevator.get();
  }

  public double getArmState() {
    return gripper.getArmTarget();
  }
  public void cubeMode(boolean cubeMode) {
    if (cubeMode) {
      gripper.cubeModeOn();
    } else {
      gripper.cubeModeOff();
    }
  }
  
  public void clawIntake() {
    gripper.wheeledClawIntake();
  }

  public void clawOuttake() {
    gripper.wheeledClawOuttake();
  }

  public void stopClawWhenSeen() {
    gripper.stopClawWhenSeen();
  }

  public void setIntakeHigh(boolean b) {
    if (b) {
      intake.retractIntake();
    } else {
      intake.extendIntake();
    }
  }

  public void setLinearSlideTarget(double t) {
    linearSlide.setTarget(t);
  }

  public void setElevatorTarget(double t) {
    elevator.setTarget(t);
  }

  public void setArmTarget(double t) {
    gripper.setSubstationMode(false);
    gripper.setArmTarget(t);
    if (t == Constants.ARM_SUBSTATION) {
      gripper.setSubstationMode(true);
    }
  }

  // public boolean intakeTargetReached() {
  //   return (0.8 * Math.abs(getIntakeState() - intake.get))
  // }

  public boolean gripperPieceDetected() {
    return gripper.pieceDetected();
  }

  public boolean linearSlideTargetReached() { // make threshold a constant later
    return (0.5 * (Math.abs(getLinearSlideState() - linearSlide.getTarget())) <= Constants.LS_THRESHOLD);
  }  

  public boolean elevatorTargetReached() { // make threshold a constant later 
    return (0.8 * (Math.abs(getElevatorState() - elevator.getTarget())) <= Constants.ELEVATOR_THRESHOLD);
  }

  public boolean armTargetReached() {
    return (0.5 * (Math.abs(getArmState() - gripper.getArmTarget())) <= Constants.ARM_THRESHOLD);
  }

  public boolean armAndElevatorReached() {
    return armTargetReached() && elevatorTargetReached();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
