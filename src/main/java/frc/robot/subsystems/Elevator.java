// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.libs.wrappers.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private static LoggedTunableNumber elevatorKp = new LoggedTunableNumber("Elevantor/kP");
  private static LoggedTunableNumber elevatorKi = new LoggedTunableNumber("Elevator/Ki");
  private static LoggedTunableNumber elevatorKd = new LoggedTunableNumber("Elevator/Kd");

  private static LoggedTunableNumber elevatorHigh = new LoggedTunableNumber("Elevator/High");
  private static LoggedTunableNumber elevatorMid = new LoggedTunableNumber("Elevator/Mid");
  private static LoggedTunableNumber elevatorHold = new LoggedTunableNumber("Elevator/Hold");
  private static LoggedTunableNumber elevatorSub = new LoggedTunableNumber("Elevator/Sub");

  static {
    elevatorKp.initDefault(0.0015);
    elevatorKi.initDefault(0);
    elevatorKd.initDefault(0);

    elevatorHigh.initDefault(0);
    elevatorMid.initDefault(0);
    elevatorHold.initDefault(0);
    elevatorSub.initDefault(0);
  }

  private static double eElevatorHigh;
  private static double eElevatorMid;
  private static double eElevatorHold;
  private static double eElevatorSub;

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

    // right.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);


    encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    elevatorPID = new PIDController(elevatorKp.get(), elevatorKi.get(), elevatorKd.get());
    right.follow(left);

    elevatorLeft = new GenericMotor(left);
    elevatorEncoder = new GenericMotor(encoder);

    upperLimitSwitch = new DigitalInput(0);

    eElevatorHigh = elevatorHigh.get();
    eElevatorMid = elevatorMid.get();
    eElevatorHold = elevatorHold.get();
    eElevatorSub = elevatorSub.get();

    if(elevatorEncoder.getSensorPose() > Constants.ELEVATOR_INTERVAL_MARKER) encoderOffset = Constants.SRX_ENCODER_TICKS;
    else encoderOffset = 0;

    target = getHigh();
    speedLimit = .8;
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

  public static double getHigh() {
    return elevatorHigh.get();
  }

  public static double getMid() {
    return elevatorMid.get();
  }

  public static double getHold() {
    return elevatorHold.get();
  }

  public static double getSub() {
    return elevatorSub.get();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator encoder offset", get());
    SmartDashboard.putNumber("elevator encoder raw", elevatorEncoder.getSensorPose());

    
    SmartDashboard.putNumber("target elevator", getTarget());
    SmartDashboard.putNumber("the difference", Math.abs(get() - getTarget()));

    SmartDashboard.putBoolean("elevator target reached", Robot.m_robotContainer.manager.elevatorTargetReached());

    if (elevatorKp.hasChanged(hashCode()) || elevatorKi.hasChanged(hashCode()) || elevatorKd.hasChanged(hashCode()) || 
        elevatorHigh.hasChanged(hashCode()) || elevatorMid.hasChanged(hashCode()) || elevatorHold.hasChanged(hashCode()) || elevatorSub.hasChanged(hashCode())) {
      elevatorPID.setP(elevatorKp.get());
      elevatorPID.setI(elevatorKi.get());
      elevatorPID.setD(elevatorKd.get());

      eElevatorHigh = elevatorHigh.get();
      eElevatorMid = elevatorMid.get();
      eElevatorHold = elevatorHold.get();
      eElevatorSub = elevatorSub.get();
    } 
  }
}