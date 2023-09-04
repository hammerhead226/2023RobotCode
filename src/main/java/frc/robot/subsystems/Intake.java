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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.swerveyshark.sharkexe.SharkExecutor;
import frc.libs.wrappers.GenericMotor;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;


public class Intake extends SubsystemBase {

  private GenericMotor roller;
  private GenericMotor intake;
  private GenericMotor intakeEncoder;

  

  private AnalogInput distanceSensor;

  
  private PIDController intakePID;

  private boolean runningOut;

  private double target;

 ;

  private double lastSpeed;

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

    roll.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100);

    pivot.setNeutralMode(NeutralMode.Coast);
    roll.setNeutralMode(NeutralMode.Brake);

    roll.setInverted(false);
    pivot.setInverted(true);

    pivot.configOpenloopRamp(0.1);
    
    encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

   

    intake = new GenericMotor(pivot);
    roller = new GenericMotor(roll);
    intakeEncoder = new GenericMotor(encoder);
    intakePID = new PIDController(Constants.INTAKE_GAINS_RETRACT[0], Constants.INTAKE_GAINS_RETRACT[1], Constants.INTAKE_GAINS_RETRACT[2]);

    target = Constants.INTAKE_RETRACT;

    intakeState = INTAKE_STATES.STOP;

    runningOut = true;

    lastSpeed = 0;

    distanceSensor = new AnalogInput(1);
 

   

    SharkExecutor.createRunnable("intake.extend", this::extendIntake);
    SharkExecutor.createRunnable("intake.runIn", this::runIn);
    SharkExecutor.createRunnable("intake.runOut", this::runOut);
    SharkExecutor.createRunnable("intake.stop", this::stop);
    SharkExecutor.createRunnable("intake.lower", this::lowerIntake);

  }

  public void run() {
    
   
    if (target == Constants.INTAKE_EXTEND) {
      intakePID.setPID(Constants.INTAKE_GAINS_EXTEND[0], Constants.INTAKE_GAINS_EXTEND[1], Constants.INTAKE_GAINS_EXTEND[2]);
    } else {
      intakePID.setPID(Constants.INTAKE_GAINS_RETRACT[0], Constants.INTAKE_GAINS_RETRACT[1], Constants.INTAKE_GAINS_RETRACT[2]);
    }

    switch (intakeState) {
      case INWARD:
        roller.set(Constants.ROLLER_RUN_SPEED);
        runningOut = false;
        break;
      case OUTWARD:
        roller.set(-0.3);
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

    double speed = intakePID.calculate(intakeEncoder.getSensorPose(), target);

    speed = clamp(speed, Constants.MAX_SPEED_DOWN, Constants.MAX_SPEED_UP);
    
   

    lastSpeed = speed;

    control(speed);

    
    
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

  
  public void lowerIntake() {
    // intakePosition = IntakePosition.LOWER;
    // intakeLowered = true;
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
   
    intakeState = INTAKE_STATES.STOP;
  }

  public void deadStop() {
    intakeState = INTAKE_STATES.DEAD_STOP;
  }
  public void control(double speed) {
    intake.set(speed);
  }

  public double getIntake() {
    return intakeEncoder.getSensorPose();
  }

  public double getTarget() {
    return target;
  }

  public void runInUntilSpike(double amps) {
    while(roller.getFalconCurrent() < amps) {
      roller.set(0.06);
    }
    stop();
  }

 

  public boolean detected() {
    return distanceSensor.getValue() >= 700 && distanceSensor.getValue() < 2500;
  }

  

  @Override
  public void periodic() {
   if(RobotContainer.intakeToggle) { 
    if (target == Constants.INTAKE_EXTEND) {
      intakePID.setPID(Constants.INTAKE_GAINS_EXTEND[0], Constants.INTAKE_GAINS_EXTEND[1], Constants.INTAKE_GAINS_EXTEND[2]);
    } else {
      intakePID.setPID(Constants.INTAKE_GAINS_RETRACT[0], Constants.INTAKE_GAINS_RETRACT[1], Constants.INTAKE_GAINS_RETRACT[2]);
    }
    switch (intakeState) {
      case INWARD:
        roller.set(Constants.ROLLER_RUN_SPEED);
        runningOut = false;
        break;
      case OUTWARD:
        roller.set(-0.3);
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

    double speed = intakePID.calculate(intakeEncoder.getSensorPose(), target);

    speed = clamp(speed, Constants.MAX_SPEED_DOWN, Constants.MAX_SPEED_UP);
    
    

    
    lastSpeed = speed;

    control(speed);
   }
    
    SmartDashboard.putNumber("intake enc", getIntake());
    SmartDashboard.putNumber("distance", distanceSensor.getValue());

    SmartDashboard.putString("intake state", intakeState.toString());

    SmartDashboard.putNumber("intake diff", Math.abs(intakeEncoder.getSensorPose() - target));

    SmartDashboard.putNumber("intake target", target);
    
    
  }
}