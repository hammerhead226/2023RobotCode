// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX elevatorLeft;
  private TalonFX elevatorRight;
  private TalonSRX elevatorEncoder; 
  private PIDController elevatorPID;
  private boolean isManual;
  private double target;

  public Elevator() {
    elevatorLeft = new TalonFX(RobotMap.ELEVATOR_MOTOR_LEFT);
    elevatorRight = new TalonFX(RobotMap.ELEVATOR_MOTOR_RIGHT);
    elevatorEncoder = new TalonSRX(RobotMap.ELEVATOR_ENCODER);

    elevatorLeft.setSelectedSensorPosition(0);
    elevatorRight.setSelectedSensorPosition(0);
    elevatorEncoder.setSelectedSensorPosition(0);

    elevatorEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    elevatorPID = new PIDController(Constants.ELEVATOR_GAINS[0], Constants.ELEVATOR_GAINS[1],
        Constants.ELEVATOR_GAINS[2]);
    elevatorRight.follow(elevatorLeft);
  }

  public void toggleManual() {
    isManual = !isManual;
  }

  public void run() {
    if (!isManual) {
      control(elevatorPID.calculate(elevatorEncoder.getSelectedSensorPosition(), target));
    } 
  }

  public void setTarget(double t) {
    target = t;
  }

  public void control(double speed) {
    elevatorLeft.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    if (isManual) {
      if (!(elevatorEncoder.getSelectedSensorPosition() <= Constants.MIN_POSITION
          || elevatorEncoder.getSelectedSensorPosition() >= Constants.MAX_POSITION))
        control(1 * Constants.ELEVATOR_COEFFICIENT);
    }
    // This method will be called once per scheduler run
  }
}
