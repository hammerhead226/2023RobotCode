// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX elevatorLeft;
  private TalonFX elevatorRight;
  private PIDController elevatorPID;
  private boolean isManual;
  private double target;

  /**
   * Edits to make to Elevator:
   * add 3 commands for Low, Mid, and High
   * this is so that the elevator will keep moving until we need it to stop
   * with the code right now, the driver will have to continuously press the
   * button to run the InstantCommand
   * create a method that stops the elevator once it gets ~10 or so ticks within
   * setpoint
   * check the shooter class from last year to get an example
   * Create a GenericEncoder object to use instead of getSelectedSensorPosition();
   * using a Magnum encoder
   * the Falcon motor needs ~5.5 rotations to rotate elevator from top to bottom
   * we are planning to create a 1:6 gear mechanism so that <1 rotation of Talon
   * moves the elevator
   * use GenericEncoder to see how many ticks rotated
   */

  public Elevator() {
    elevatorLeft = new TalonFX(RobotMap.ELEVATOR_MOTOR_LEFT);
    elevatorRight = new TalonFX(RobotMap.ELEVATOR_MOTOR_RIGHT);

    elevatorLeft.setSelectedSensorPosition(0);
    elevatorRight.setSelectedSensorPosition(0);

    elevatorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    elevatorPID = new PIDController(Constants.ELEVATOR_GAINS[0], Constants.ELEVATOR_GAINS[1],
        Constants.ELEVATOR_GAINS[2]);
    elevatorRight.follow(elevatorLeft);
  }

  public void toggleManual() {
    isManual = !isManual;
  }

  public void run() {
    if (!isManual) {
      control(elevatorPID.calculate(elevatorLeft.getSelectedSensorPosition(), target));
    } else {
      control(0);
    }

  }

  public void setTarget(double t) {
    target = t;
  }

  public void control(double speed) {
    // -1 to 1
    elevatorLeft.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    if (isManual) {
      if (!(elevatorLeft.getSelectedSensorPosition() <= Constants.MIN_POSITION
          || elevatorLeft.getSelectedSensorPosition() >= Constants.MAX_POSITION))
        control(1 * Constants.ELEVATOR_COEFFICIENT);
    }
    // This method will be called once per scheduler run
  }
}
