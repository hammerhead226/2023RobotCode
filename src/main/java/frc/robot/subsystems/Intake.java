// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.swerveyshark.sharkexe.SharkExecutor;
import frc.libs.wrappers.GenericMotor;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

  private GenericMotor roller;
  private TalonFX intake;
  private GenericMotor intakeEncoder;

  private AnalogInput distanceSensor;


  private boolean runningOut;

  private double target;


  private enum INTAKE_STATES{
    INWARD,
    OUTWARD,
    DEAD_STOP,
    STOP
  }

  private INTAKE_STATES intakeState;



  public Intake() {
    TalonFX pivot = new TalonFX(RobotMap.INTAKE_PORT, Constants.CANBUS);
    TalonFX roll = new TalonFX(RobotMap.ROLLER_PORT, Constants.CANBUS);
    TalonSRX encoder = new TalonSRX(RobotMap.INTAKE_ENCODER_PORT);


    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();


    // TODO:: probably going to have to retune this stuff
    intakeConfig.slot0.kP = 0.0005;
    intakeConfig.slot0.kI = 0.00005;
    intakeConfig.slot0.kD = 0;
    intakeConfig.slot0.kF = 0;


    // to be changed later
    intakeConfig.slot1.kP = 0.00046;
    intakeConfig.slot1.kI = 0.00032;
    intakeConfig.slot1.kD = 0;
    intakeConfig.slot1.kF = 0;


    intakeConfig.neutralDeadband = 0.04;

    intakeConfig.motionCruiseVelocity = 300;
    intakeConfig.motionAcceleration = 150;
    

    pivot.configAllSettings(intakeConfig);


    roll.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100);

    pivot.setNeutralMode(NeutralMode.Coast);
    roll.setNeutralMode(NeutralMode.Brake);

    roll.setInverted(false);
    pivot.setInverted(true);

    encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    intake = pivot;
    roller = new GenericMotor(roll);
    intakeEncoder = new GenericMotor(encoder);

    target = Constants.INTAKE_RETRACT;

    intakeState = INTAKE_STATES.STOP;

    runningOut = true;
 

    SharkExecutor.createRunnable("intake.extend", this::extendIntake);
    SharkExecutor.createRunnable("intake.runIn", this::runIn);
    SharkExecutor.createRunnable("intake.runOut", this::runOut);
    SharkExecutor.createRunnable("intake.stop", this::stop);
  }

  public void run() {
    intake.setSelectedSensorPosition(getIntake());

    if (target == Constants.INTAKE_RETRACT) {
      // TODO:: determine primary or aux later
      intake.selectProfileSlot(0, 0);
    } else {
      intake.selectProfileSlot(1, 0);
    }

    switch (intakeState) {
      case INWARD:
        roller.set(Constants.ROLLER_RUN_SPEED);
        runningOut = false;
        break;
      case OUTWARD:
        roller.set(-0.2);
        runningOut = true;
        break;
      case DEAD_STOP:
        if (Math.abs(intakeEncoder.getSensorPose() - target) >= 300) {
          roller.set(0.15);
        } else {
          intakeState = INTAKE_STATES.STOP;
          }
        break; 
      case STOP:
        roller.set(0);
        break;
      default:
        SmartDashboard.putString("deez", "nuts");
        break;
    }
    

    goTo(target);
  }

  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(value, max));
  }

  public void extendIntake() {
    target = Constants.INTAKE_EXTEND;
  }

  public void retractIntake() {
    target = Constants.INTAKE_RETRACT;
  }

  // Roller Methods
  public void runIn() {
    intakeState = INTAKE_STATES.INWARD;
  }

  public void runOut() {
    intakeState = INTAKE_STATES.OUTWARD;
    target = Constants.INTAKE_OUTTAKE;
  }

  public void stop() {
    // roller.set(0);
    intakeState = INTAKE_STATES.STOP;
  }

  public void deadStop() {
    intakeState = INTAKE_STATES.DEAD_STOP;
  }
  public void control(double speed) {
    intake.set(ControlMode.PercentOutput, speed);
  }

  public void goTo(double target) {
    intake.set(ControlMode.MotionMagic, target);
  }

  public double getIntake() {
    return intakeEncoder.getSensorPose();
  }

  public double getTarget() {
    return target;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake enc", getIntake());

    SmartDashboard.putString("intake state", intakeState.toString());

    SmartDashboard.putNumber("intake diff", Math.abs(intakeEncoder.getSensorPose() - target));

    SmartDashboard.putNumber("intake target", target);

    SmartDashboard.putNumber("intake motor percent", intake.getMotorOutputPercent());
  
  }
}