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
  private TalonFX elevatorTalon1;
  private TalonFX elevatorTalon2;
  private PIDController pid;
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
    elevatorTalon1 = new TalonFX(RobotMap.ELEVATOR_MOTOR_1);
    elevatorTalon2 = new TalonFX(RobotMap.ELEVATOR_MOTOR_2);

    elevatorTalon1.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    elevatorTalon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    elevatorTalon1.configFeedbackNotContinuous(true, 0);

    pid = new PIDController(Constants.ELEVATOR_GAINS[0], Constants.ELEVATOR_GAINS[1], Constants.ELEVATOR_GAINS[2]);
    
    elevatorTalon2.follow(elevatorTalon1);
    elevatorTalon2.setInverted(true);

    //For Testing
    initialPos = elevatorTalon1.getSelectedSensorPosition(); 
    gen = new GenericEncoder(input, 0, 2048, 0);
  }

  public void toggleManual(){
    isManual = !isManual; 
  }


  public void runManual(double speed)
  {
    if(isManual){
      if(!(elevatorTalon1.getSelectedSensorPosition() <= Constants.MIN_POSITION || elevatorTalon1.getSelectedSensorPosition() >= Constants.MAX_POSITION))
        elevatorTalon1.set(ControlMode.PercentOutput, speed * Constants.ELEVATOR_COEFFICIENT);
    }
  }

  public void setLow()
  {
    elevatorTalon1.set(ControlMode.PercentOutput, pid.calculate(elevatorTalon1.getSelectedSensorPosition(), Constants.LOW_SETPOINT));
    elevatorTalon2.set(ControlMode.PercentOutput, pid.calculate(elevatorTalon1.getSelectedSensorPosition(), Constants.LOW_SETPOINT));
  }

  public void setMid()
  {
    elevatorTalon1.set(ControlMode.PercentOutput, pid.calculate(elevatorTalon1.getSelectedSensorPosition(), Constants.MID_SETPOINT));
    elevatorTalon2.set(ControlMode.PercentOutput, pid.calculate(elevatorTalon1.getSelectedSensorPosition(), Constants.MID_SETPOINT));
  }

  public void setHigh()
  {
    elevatorTalon1.set(ControlMode.PercentOutput, pid.calculate(elevatorTalon1.getSelectedSensorPosition(), Constants.HIGH_SETPOINT));
    elevatorTalon2.set(ControlMode.PercentOutput, pid.calculate(elevatorTalon1.getSelectedSensorPosition(), Constants.HIGH_SETPOINT));
  }

  //Potentially make one function and use lambdas to make simpler
  public void setTo(double setpoint){
    elevatorTalon1.set(ControlMode.PercentOutput, pid.calculate(elevatorTalon1.getSelectedSensorPosition(), initialPos + setpoint));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
