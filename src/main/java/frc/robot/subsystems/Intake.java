// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.Rev2mDistanceSensor.Port;
// import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
// import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
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

  // private Rev2mDistanceSensor distanceSensor;

  // private String intakePosition;

  private boolean intakeOn;
  private boolean intakeLowered;
  private PIDController intakePID;

  private double target;

  private boolean intakeExtended;

  boolean pivot_brake = Constants.INTAKE_PIVOT_BRAKE_DEFAULT;
    GenericEntry pivot_brake_toggle = Shuffleboard.getTab(getName())
    .add("Enable Brake Mode", pivot_brake)
    .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();


  public Intake() {
    TalonFX pivot = new TalonFX(RobotMap.INTAKE_PORT, Constants.CANBUS);
    TalonFX roll = new TalonFX(RobotMap.ROLLER_PORT, Constants.CANBUS);
    TalonSRX encoder = new TalonSRX(RobotMap.INTAKE_ENCODER_PORT);

    roll.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100);

    
    roll.setNeutralMode(NeutralMode.Coast);

    roll.setInverted(false);
    
    encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    // distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

    // distanceSensor.setRangeProfile(RangeProfile.kHighSpeed);
    // distanceSensor.setDistanceUnits(Unit.kInches);

    intake = new GenericMotor(pivot);
    roller = new GenericMotor(roll);
    intakeEncoder = new GenericMotor(encoder);
    intakePID = new PIDController(Constants.INTAKE_GAINS[0], Constants.INTAKE_GAINS[1], Constants.INTAKE_GAINS[2]);

    intakeExtended = true;
 
    SharkExecutor.createRunnable("intake.extend", this::extendIntake);
    SharkExecutor.createRunnable("intake.runIn", this::runIn);
    SharkExecutor.createRunnable("intake.runOut", this::runOut);
    SharkExecutor.createRunnable("intake.stop", this::stop);
    SharkExecutor.createRunnable("intake.lower", this::lowerIntake);

    
    pivot.setNeutralMode(pivot_brake ? NeutralMode.Brake : NeutralMode.Coast);

  }

  public void run() {
    
    // if(Robot.m_robotContainer.gripper.getCubeMode()
    //  || Robot.m_robotContainer.gripper.getArmTarget() == Constants.ARM_SCORE
    // //  || Robot.m_robotContainer.elevator.getTarget() != Constants.ELEVATOR_HOLD
    //  ) {
    //   intakeOn = true;
    // }

    target = intakeExtended == true ? Constants.INTAKE_EXTEND : Constants.INTAKE_RETRACT;

    double speed = intakePID.calculate(intakeEncoder.getSensorPose(), target);

    speed = clamp(speed, Constants.MAX_SPEED_DOWN, Constants.MAX_SPEED_UP);

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
    intakeExtended = true;
  }

  public void retractIntake() {
    intakeExtended = false;
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
      if (!detected()) {
        roller.set(Constants.ROLLER_RUN_SPEED);
      } else {
        stop();
      }
      
    // }
  }

  public void runOut() {
    // if (intakeOn) {
      roller.set(-0.7);
    // }
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

  public double getTarget() {
    return target;
  }

  public void setDistanceSensor(boolean bool) {
    // distanceSensor.setEnabled(bool);
  }

  public void setDistanceSensorAuto(boolean bool) {
    // distanceSensor.setAutomaticMode(bool);
  }

  public boolean detected() {
    // return distanceSensor.getRange() <= 19 && distanceSensor.getRange() > 0;
  return true;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("intake enc", getIntake());

    // SmartDashboard.putNumber("limelight stuff", LimeLight.getHorizontalOffset());
    // SmartDashboard.putNumber("limelight stuff 2", LimeLight.getValue());

    if (pivot_brake_toggle.getBoolean(Constants.INTAKE_PIVOT_BRAKE_DEFAULT) != pivot_brake) {
      pivot_brake = pivot_brake_toggle.getBoolean(Constants.INTAKE_PIVOT_BRAKE_DEFAULT);
      intake.getFalcon().setNeutralMode(pivot_brake ? NeutralMode.Brake : NeutralMode.Coast);
      
    }
    
  }
}