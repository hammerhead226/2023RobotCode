// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.wrappers.GenericEncoder;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX elevator1;
  private TalonFX elevator2;
  private PIDController elevatorPID;
  private boolean isManual; 
  private double initialPos;
  private GenericEncoder gen;
  private AnalogInput input = new AnalogInput(0);
  /**
   * Edits to make to Elevator:
   * add 3 commands for Low, Mid, and High
    * this is so that the elevator will keep moving until we need it to stop
    * with the code right now, the driver will have to continuously press the button to run the InstantCommand
   * create a method that stops the elevator once it gets ~10 or so ticks within setpoint
    * check the shooter class from last year to get an example
   * Create a GenericEncoder object to use instead of getSelectedSensorPosition();
    * using a Magnum encoder
    * the Falcon motor needs ~5.5 rotations to rotate elevator from top to bottom
    * we are planning to create a 1:6 gear mechanism so that <1 rotation of Talon moves the elevator
    * use GenericEncoder to see how many ticks rotated
   */

  public Elevator() {
    elevator1 = new TalonFX(RobotMap.ELEVATOR_MOTOR_1);
    elevator2 = new TalonFX(RobotMap.ELEVATOR_MOTOR_2);

    elevator1.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    elevator1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    elevatorPID = new PIDController(Constants.ELEVATOR_GAINS[0], Constants.ELEVATOR_GAINS[1], Constants.ELEVATOR_GAINS[2]);

    elevator1.setInverted(Constants.ELEVATOR_MOTOR_1_INVERT);
    elevator2.setInverted(Constants.ELEVATOR_MOTOR_2_INVERT);

    gen = new GenericEncoder(input, 0, 2048, 0);
  }

  public void toggleManual(){
    isManual = !isManual; 
  }


  public void runManual(double speed)
  {
    if(isManual){
      if(!(elevator1.getSelectedSensorPosition() <= Constants.MIN_POSITION || elevator1.getSelectedSensorPosition() >= Constants.MAX_POSITION))
        elevator1.set(ControlMode.PercentOutput, speed * Constants.ELEVATOR_COEFFICIENT);
    }
  }

  public void setLow()
  {
    elevator1.set(ControlMode.PercentOutput, elevatorPID.calculate(elevator1.getSelectedSensorPosition(), Constants.LOW_SETPOINT));
    elevator2.set(ControlMode.PercentOutput, elevatorPID.calculate(elevator1.getSelectedSensorPosition(), Constants.LOW_SETPOINT));
  }

  public void setMid()
  {
    elevator1.set(ControlMode.PercentOutput, elevatorPID.calculate(elevator1.getSelectedSensorPosition(), Constants.MID_SETPOINT));
    elevator2.set(ControlMode.PercentOutput, elevatorPID.calculate(elevator1.getSelectedSensorPosition(), Constants.MID_SETPOINT));
  }

  public void setHigh()
  {
    elevator1.set(ControlMode.PercentOutput, elevatorPID.calculate(elevator1.getSelectedSensorPosition(), Constants.HIGH_SETPOINT));
    elevator2.set(ControlMode.PercentOutput, elevatorPID.calculate(elevator1.getSelectedSensorPosition(), Constants.HIGH_SETPOINT));
  }

  //Potentially make one function and use lambdas to make simpler
  public void setTo(double setpoint){
    elevator1.set(ControlMode.PercentOutput, elevatorPID.calculate(elevator1.getSelectedSensorPosition(), initialPos + setpoint));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
