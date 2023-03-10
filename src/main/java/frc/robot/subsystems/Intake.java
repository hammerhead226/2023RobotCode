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
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.wrappers.GenericMotor;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

  private GenericMotor roller;
  private GenericMotor intake;
  private GenericMotor intakeEncoder;
  private boolean intakeOn;
  private PIDController intakePID;

  public Intake() {
    TalonFX iFalcon = new TalonFX(RobotMap.INTAKE_PORT, Constants.CANBUS);
    TalonFX roll = new TalonFX(RobotMap.ROLLER_PORT, Constants.CANBUS);
    TalonSRX encoder = new TalonSRX(RobotMap.INTAKE_ENCODER_PORT);

    roll.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100);

    iFalcon.setNeutralMode(NeutralMode.Brake);
    roll.setNeutralMode(NeutralMode.Coast);

    roll.setInverted(true);
    
    encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    intake = new GenericMotor(iFalcon);
    roller = new GenericMotor(roll);
    intakeEncoder = new GenericMotor(encoder);
    intakePID = new PIDController(Constants.INTAKE_GAINS[0], Constants.INTAKE_GAINS[1], Constants.INTAKE_GAINS[2]);
  }

  public void run() {
    if (intakeOn) {
      double extendSpeed = intakePID.calculate(intakeEncoder.getSensorPose(), Constants.INTAKE_EXTEND);
      if (extendSpeed > Constants.MAX_SPEED_UP) {
        extendSpeed = Constants.MAX_SPEED_UP;
      }
      control(extendSpeed);
    } else {
      double retractSpeed = intakePID.calculate(intakeEncoder.getSensorPose(), Constants.INTAKE_RETRACT);
      if (retractSpeed > Constants.MAX_SPEED_DOWN){
        retractSpeed = Constants.MAX_SPEED_DOWN;
      }
      control(retractSpeed);
    }

  }

  public void toggleIntake() {
    intakeOn = !intakeOn;
  }

  // Roller Methods
  public void runIn() {
    if (intakeOn) {
      roller.set(Constants.ROLLER_RUN_SPEED);
    }
  }

  public void runOut() {
    if (intakeOn) {
      roller.set(-Constants.ROLLER_RUN_SPEED);
    }
  }

  public void stop() {
    roller.set(0);
  }

  public void control(double speed) {
    intake.set(speed);
  }

  public double getIntake() {
    return intakeEncoder.getSensorPose();
  }

  @Override
  public void periodic() {}
}
