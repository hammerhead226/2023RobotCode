// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX elevatorTalon1;
  private TalonFX elevatorTalon2;
  private PIDController pid;
  private boolean isManual; 
  private double initialPos;

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
    elevatorTalon1.set(ControlMode.PercentOutput, pid.calculate(elevatorTalon1.getSelectedSensorPosition(), Constants.LOW_SETPOINT));
  }

  public void setMid()
  {
    elevatorTalon1.set(ControlMode.PercentOutput, pid.calculate(elevatorTalon1.getSelectedSensorPosition(), Constants.MID_SETPOINT));
    elevatorTalon1.set(ControlMode.PercentOutput, pid.calculate(elevatorTalon1.getSelectedSensorPosition(), Constants.MID_SETPOINT));
  }

  public void setHigh()
  {
    elevatorTalon1.set(ControlMode.PercentOutput, pid.calculate(elevatorTalon1.getSelectedSensorPosition(), Constants.HIGH_SETPOINT));
    elevatorTalon1.set(ControlMode.PercentOutput, pid.calculate(elevatorTalon1.getSelectedSensorPosition(), Constants.HIGH_SETPOINT));
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
