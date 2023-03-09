// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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

  private double encoderOffset;

  public Elevator() {
    elevatorLeft = new TalonFX(RobotMap.ELEVATOR_MOTOR_LEFT, Constants.CANBUS);
    elevatorRight = new TalonFX(RobotMap.ELEVATOR_MOTOR_RIGHT, Constants.CANBUS);
    elevatorEncoder = new TalonSRX(RobotMap.ELEVATOR_ENCODER);

    elevatorLeft.setNeutralMode(NeutralMode.Brake);
    elevatorRight.setNeutralMode(NeutralMode.Brake);

    elevatorLeft.setInverted(true);
    elevatorRight.setInverted(true);


    elevatorEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    elevatorPID = new PIDController(Constants.ELEVATOR_GAINS[0], Constants.ELEVATOR_GAINS[1],
        Constants.ELEVATOR_GAINS[2]);
    elevatorRight.follow(elevatorLeft);

    if(elevatorEncoder.getSelectedSensorPosition() > Constants.ELEVATOR_INTERVAL_MARKER) encoderOffset = Constants.SRX_ENCODER_TICKS;
    else encoderOffset = 0;

    target = 0;
  }

  public void toggleManual() {
    isManual = !isManual;
  }

  public double get() {
    return elevatorEncoder.getSelectedSensorPosition() - encoderOffset;
  }
 
  public void run() {
    double elevatorTarget = target;
    if (elevatorTarget <= -300) {
      elevatorTarget = -300;
    } else if (elevatorTarget >= 2000) {
      elevatorTarget = 2000;
    }
    control(elevatorPID.calculate(get(), elevatorTarget));
  }

  public void setTarget(double t) {
    target = t;
  }

  public void control(double speed) {
    elevatorLeft.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {}
}
