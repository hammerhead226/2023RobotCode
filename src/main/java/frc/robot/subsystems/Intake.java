// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  // private String intakePosition;

  private boolean intakeOn;
  private boolean intakeLowered;
  private PIDController intakePID;

  private double target;

  private boolean intakeTucked;

  private enum IntakePosition {
    RETRACT,
    EXTEND,
    LOWER
  }

  private IntakePosition intakePosition;

  public Intake() {
    CANSparkMax intakeNeo = new CANSparkMax(RobotMap.INTAKE_PORT, MotorType.kBrushless);
    TalonFX roll = new TalonFX(RobotMap.ROLLER_PORT, Constants.CANBUS);
    TalonSRX encoder = new TalonSRX(RobotMap.INTAKE_ENCODER_PORT);

    roll.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100);

    intakeNeo.setIdleMode(IdleMode.kBrake);
    roll.setNeutralMode(NeutralMode.Coast);

    roll.setInverted(false);
    
    encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    intake = new GenericMotor(intakeNeo);
    roller = new GenericMotor(roll);
    intakeEncoder = new GenericMotor(encoder);
    intakePID = new PIDController(Constants.INTAKE_GAINS[0], Constants.INTAKE_GAINS[1], Constants.INTAKE_GAINS[2]);
    intakeOn = false;
    intakeTucked = false;

    //TODO:: change this later 
    intakePosition = IntakePosition.LOWER;
  }

  public void run() {
    
    // if(Robot.m_robotContainer.gripper.getCubeMode()
    //  || Robot.m_robotContainer.gripper.getArmTarget() == Constants.ARM_SCORE
    // //  || Robot.m_robotContainer.elevator.getTarget() != Constants.ELEVATOR_HOLD
    //  ) {
    //   intakeOn = true;
    // }
    switch (intakePosition){
      case EXTEND:
        target = Constants.INTAKE_EXTEND;
        double extendSpeed = intakePID.calculate(intakeEncoder.getSensorPose(), target);
        if (extendSpeed > Constants.MAX_SPEED_UP) {
          extendSpeed = Constants.MAX_SPEED_UP;
        }
        control(extendSpeed);
        break;

      case RETRACT:
        target = Constants.INTAKE_RETRACT;
        double retractSpeed = intakePID.calculate(intakeEncoder.getSensorPose(), target);
        if (Math.abs(retractSpeed) > Constants.MAX_SPEED_DOWN){
          retractSpeed = -Constants.MAX_SPEED_DOWN;
        }
        control(retractSpeed);
        break;
      case LOWER:
        target = Constants.INTAKE_LOWERED;
        double lowerSpeed = intakePID.calculate(intakeEncoder.getSensorPose(), target);
        control(lowerSpeed);
        break;
      default:
        SmartDashboard.putString("cry about it", "cry about it");
    }

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

  // public void toggleIntake() {
  //   intakeOn = !intakeOn;
  // }

  public void extendIntake() {
    intakePosition = IntakePosition.EXTEND;
    // intakeOn = true;
  }

  public void retractIntake() {
    intakePosition = IntakePosition.RETRACT;
    // intakeOn = false;
  }

  // public void toggleLowerIntake() {
  //   intakeLowered = !intakeLowered;
  // }

  public void lowerIntake() {
    intakePosition = IntakePosition.LOWER;
    // intakeLowered = true;
  }

  // Roller Methods
  public void runIn() {
    // if (intakeOn) {
      roller.set(Constants.ROLLER_RUN_SPEED);
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake enc", getIntake());
    SmartDashboard.putNumber("limelight stuff", LimeLight.getHorizontalOffset());
    SmartDashboard.putNumber("limelight stuff 2", LimeLight.getValue());
  }
}