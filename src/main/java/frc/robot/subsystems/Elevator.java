// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.wrappers.GenericMotor;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private GenericMotor elevatorLeft;
  private GenericMotor elevatorRight;
  private GenericMotor elevatorEncoder; 

  private DigitalInput upperLimitSwitch;


  private PIDController elevatorPID;
  private boolean isManual;
  private double target;

  private double speedLimit;

  private double encoderOffset;

  public Elevator() {
    TalonFX left = new TalonFX(RobotMap.ELEVATOR_MOTOR_LEFT, Constants.CANBUS);
    TalonFX right = new TalonFX(RobotMap.ELEVATOR_MOTOR_RIGHT, Constants.CANBUS);
    TalonSRX encoder = new TalonSRX(RobotMap.ELEVATOR_ENCODER);

    left.setNeutralMode(NeutralMode.Brake);
    right.setNeutralMode(NeutralMode.Brake);

    left.setInverted(true);
    right.setInverted(true);

    right.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100);


    encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    elevatorPID = new PIDController(Constants.ELEVATOR_GAINS[0], Constants.ELEVATOR_GAINS[1],
        Constants.ELEVATOR_GAINS[2]);
    right.follow(left);

    elevatorLeft = new GenericMotor(left);
    elevatorEncoder = new GenericMotor(encoder);

    upperLimitSwitch = new DigitalInput(0);

    if(elevatorEncoder.getSensorPose() > Constants.ELEVATOR_INTERVAL_MARKER) encoderOffset = Constants.SRX_ENCODER_TICKS;
    else encoderOffset = 0;

    target = Constants.ELEVATOR_HIGH;
    speedLimit = .76;
  }

  public void toggleManual() {
    isManual = !isManual;
  }

  public double get() {
    return elevatorEncoder.getSensorPose() - encoderOffset;
  }
 
  public void run() {
    

    double motorSpeed = elevatorPID.calculate(get(), target);

    if(motorSpeed > speedLimit) motorSpeed = speedLimit;
    else if(motorSpeed < -speedLimit) motorSpeed = -speedLimit;

    if(!upperLimitSwitch.get()) {
      motorSpeed = 0;
    }

    control(motorSpeed);
    SmartDashboard.putNumber("motor sped", motorSpeed);
  }

  public void setTarget(double t) {
    target = t;
  }

  public double getTarget() {
    return target;
  }

  public void control(double speed) {
    elevatorLeft.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator encoder offset", get());
    SmartDashboard.putNumber("elevator encoder raw", elevatorEncoder.getSensorPose());

  }
}
