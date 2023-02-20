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

public class Intake extends SubsystemBase {

  private TalonFX roller;
  private TalonFX intake;
  private TalonSRX intakeEncoder;
  private boolean intakeOn;
  private PIDController intakePID;

  /**
   * Creates a new Intake.
   * instead of using Commands, we can use run {} methods
   */
  public Intake() {
    intake = new TalonFX(RobotMap.INTAKE_PORT);
    roller = new TalonFX(RobotMap.ROLLER_PORT);
    intakeEncoder = new TalonSRX(RobotMap.INTAKE_ENCODER_PORT);
    intake.setNeutralMode(NeutralMode.Brake);
    roller.setNeutralMode(NeutralMode.Coast);

    intakeEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    intakePID = new PIDController(Constants.INTAKE_GAINS[0], Constants.INTAKE_GAINS[1], Constants.INTAKE_GAINS[2]);
  }

  public void run() {
    if (intakeOn) {
      control(intakePID.calculate(intakeEncoder.getSelectedSensorPosition(), Constants.INTAKE_EXTEND));

    } else {
      control(intakePID.calculate(intakeEncoder.getSelectedSensorPosition(), Constants.INTAKE_RETRACT));
    }

  }

  public void toggleIntake() {
    intakeOn = !intakeOn;
  }

  // Roller Methods
  public void runIn() {
    control(Constants.ROLLER_RUN_SPEED);
  }

  public void runOut() {
    control(-Constants.ROLLER_RUN_SPEED);
  }

  public void stop() {
    control(0);
  }

  public void control(double speed) {
    intake.set(ControlMode.PercentOutput, speed);
  }

  public double getIntake() {
    return intakeEncoder.getMotorOutputPercent();
  }

  public double getRoller() {
    return roller.getMotorOutputPercent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
