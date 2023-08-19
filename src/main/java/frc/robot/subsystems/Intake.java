// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Date;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.swerveyshark.sharkexe.SharkExecutor;
import frc.libs.wrappers.GenericMotor;
import frc.libs.wrappers.LimeLight;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.LimelightLineUp;

public class Intake extends SubsystemBase {

  private GenericMotor roller;
  private GenericMotor intake;
  private GenericMotor intakeEncoder;

  // private Rev2mDistanceSensor dist;

  private AnalogInput distanceSensor;

  // private String intakePosition;

  private boolean intakeOn;
  private boolean intakeLowered;
  private PIDController intakePID;

  private boolean runningOut;

  private double target;

  private boolean intakeExtended;

  private double lastSpeed;

  private enum INTAKE_STATES{
    INWARD,
    OUTWARD,
    DEAD_STOP,
    STOP
  }

  private INTAKE_STATES intakeState;

  private boolean runInward;

  private boolean runOutward;


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

    // dist = new Rev2mDistanceSensor(Port.kOnboard);

    // dist.setRangeProfile(RangeProfile.kHighSpeed);
    // dist.setDistanceUnits(Unit.kInches);

    intake = new GenericMotor(pivot);
    roller = new GenericMotor(roll);
    intakeEncoder = new GenericMotor(encoder);
    intakePID = new PIDController(Constants.INTAKE_GAINS_RETRACT[0], Constants.INTAKE_GAINS_RETRACT[1], Constants.INTAKE_GAINS_RETRACT[2]);

    target = Constants.INTAKE_EXTEND;

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
    
    // if(Robot.m_robotContainer.gripper.getCubeMode()
    //  || Robot.m_robotContainer.gripper.getArmTarget() == Constants.ARM_SCORE
    // //  || Robot.m_robotContainer.elevator.getTarget() != Constants.ELEVATOR_HOLD
    //  ) {
    //   intakeOn = true;
    // }
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
    
    boolean condition = (speed > 0 && lastSpeed < 0) || (speed < 0 && lastSpeed > 0);

    // if (intake.getFalconCurrent() >= 35 && !condition) {
    //   speed = 0;
    // }

    lastSpeed = speed;

    control(speed);

    
    // if ()
    // switch (intakePosition){
    //   case EXTEND:
    //     target = Constants.INTAKE_EXTEND;
    //     double extendSpeed = intakePID.calculate(intakeEncoder.getSensorPose(), target);
    //     if (Math.abs(extendSpeed) > Constants.MAX_SPEED_UP) {
    //       extendSpeed = Constants.MAX_SPEED_UP;
    //     }
    //     control(extendSpeed);
    //     break;

    //   case RETRACT:
    //     target = Constants.INTAKE_RETRACT;
    //     double retractSpeed = intakePID.calculate(intakeEncoder.getSensorPose(), target);
    //     if (Math.abs(retractSpeed) > Constants.MAX_SPEED_DOWN){
    //       retractSpeed = -Constants.MAX_SPEED_DOWN;
    //     }
    //     control(retractSpeed);
    //     break;
    //   case LOWER:
    //     target = Constants.INTAKE_LOWERED;
    //     double lowerSpeed = intakePID.calculate(intakeEncoder.getSensorPose(), target);
    //     control(lowerSpeed);
    //     break;
    //   default:
    //     SmartDashboard.putString("cry about it", "cry about it");
    // }

    // if (intakeLowered){
    //   target = Constants.INTAKE_LOWERED;
    //   double lowerSpeed = intakePID.calculate(intakeEncoder.getSensorPose(), target);
    //   control(lowerSpeed);
    // } else {
    //     if (intakeOn) {
    //       target = Constants.INTAKE_EXTEND;
    //       double extendSpeed = intakePID.calculate(intakeEncoder.getSensorPose(), target);
    //       if (extendSpeed > Constants.MAX_SPEED_UP) {
    //         extendSpeed = Constants.MAX_SPEED_UP;
    //       }
    //       control(extendSpeed);
    //     } else {
    //       target = Constants.INTAKE_RETRACT;
    //       double retractSpeed = intakePID.calculate(intakeEncoder.getSensorPose(), target);
    //       if (Math.abs(retractSpeed) > Constants.MAX_SPEED_DOWN){
    //         retractSpeed = -Constants.MAX_SPEED_DOWN;
    //       }
    //       control(retractSpeed);
    //       SmartDashboard.putNumber("intake speed", retractSpeed);
    //       // SmartDashboard.putNumber("intake pose", intakeEncoder.getSensorPose());
    //     }
    // }
     
    // SmartDashboard.putNumber("intake stator", intake.getFalcon().getStatorCurrent());
    // SmartDashboard.putNumber("intake supply", intake.getFalcon().getSupplyCurrent());
  }

  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(value, max));
  }

  // public void toggleIntake() {
  //   intakeOn = !intakeOn;
  // }

  public void extendIntake() {
    target = Constants.INTAKE_EXTEND;
  }

  public void retractIntake() {
    target = Constants.INTAKE_RETRACT;
  }

  // public void toggleLowerIntake() {
  //   intakeLowered = !intakeLowered;
  // }

  public void lowerIntake() {
    // intakePosition = IntakePosition.LOWER;
    // intakeLowered = true;
  }

  // Roller Methods
  public void runIn() {
    // if (intakeOn) {
      // extendIntake();
      intakeState = INTAKE_STATES.INWARD;
      // roller.set(Constants.ROLLER_RUN_SPEED);
      // runningOut = false;
      // Robot.m_robotContainer.manip.get
      // if (!detected()) {
      //   roller.set(Constants.ROLLER_RUN_SPEED);
      // } else {
      //   stop();
      // }
      
    // }
  }

  public void runOut() {
    // if (intakeOn) {
      // runOutward = true;
      intakeState = INTAKE_STATES.OUTWARD;
      target = Constants.INTAKE_OUTTAKE;
      // roller.set(-0.3);
      // runningOut = true;
    // }
  }

  public void stop() {
    // roller.set(0);
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

  // public void setDistanceSensor(boolean bool) {
    // dist.setEnabled(bool);
  // }

  // public void setDistanceSensorAuto(boolean bool) {
    // dist.setAutomaticMode(bool);
  // }

  public boolean detected() {
    return distanceSensor.getValue() >= 700 && distanceSensor.getValue() < 2500;
  }

  // public boolean detectedDelay() {
  //   return detected() && 
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake enc", getIntake());
    SmartDashboard.putNumber("distance", distanceSensor.getValue());

    SmartDashboard.putString("intake state", intakeState.toString());

    SmartDashboard.putNumber("intake diff", Math.abs(intakeEncoder.getSensorPose() - target));

    SmartDashboard.putNumber("intake target", target);
    
    // if (detected() && !runningOut) { 
      
    // //   // runInUntilSpike(1.3);
    
    //   retractIntake();
    //   // deadStop();
    // //   // Robot.m_robotContainer.manager.
    // //   // deadStop();
    
      
      

    // } 
    // SmartDashboard.putNumber("limelight stuff", LimeLight.getHorizontalOffset());
    // SmartDashboard.putNumber("limelight stuff 2", LimeLight.getValue());
  }
}