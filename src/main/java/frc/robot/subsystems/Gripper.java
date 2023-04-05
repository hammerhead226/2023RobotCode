// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.wrappers.GenericMotor;
import frc.libs.wrappers.GenericMotor.PassiveMode;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Gripper extends SubsystemBase {
  /**
   * Three servos in total
   * 1 NEO motor as the arm
   * 1 NEO motor to open/close the gripper
   * 1 servo to rotate hand
   * up and down position for arm (toggle) --> A button
   * pincing (toggle) --> B button
   * wrist rotation (180 or 0 degrees; also toggle) --> X button
   */

  // private TalonFX wrist;
  private CANSparkMax wrist;
  private GenericMotor arm;
  private GenericMotor claw; 

  private PIDController armPID;
  private PIDController clawPID;
  private PIDController wristPID;

  // private Rev2mDistanceSensor distanceSensor;

  private AnalogInput proximitySensor;

  private boolean wristToggle = true;
  private boolean armToggle = true;

  private double armTarget = 0;

  private boolean clawToggle = true;

  private boolean cubeMode = false;

  private double armSpeedLimit;

  private boolean substationMode = false;

  public Gripper() {
    // wrist = new CANSparkMax(RobotMap.GRIPPER_WRIST, MotorType.kBrushless);
    claw = new GenericMotor(new TalonFX(RobotMap.CLAW_MOTOR, Constants.CANBUS));
    arm = new GenericMotor(new TalonFX(RobotMap.ARM_MOTOR, Constants.CANBUS));

    // distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

    // wrist.setIdleMode(IdleMode.kBrake);
    claw.setNeutralMode(PassiveMode.BRAKE);
    arm.setNeutralMode(PassiveMode.BRAKE);

    armPID = new PIDController(Constants.ARM_GAINS[0], Constants.ARM_GAINS[1], Constants.ARM_GAINS[2]);
    clawPID = new PIDController(Constants.CLAW_GAINS[0], Constants.CLAW_GAINS[1], Constants.CLAW_GAINS[2]);
    wristPID = new PIDController(Constants.WRIST_GAINS[0], Constants.WRIST_GAINS[1], Constants.WRIST_GAINS[2]);
    

    // distanceSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kInches, RangeProfile.kHighAccuracy);
    armSpeedLimit = 0.8;
    this.proximitySensor = new AnalogInput(0);
  }

  public void run() {
    if(substationMode)
      closeClawWhenSeen();

    if(clawToggle) {
      if(cubeMode) {
        claw.set( clawPID.calculate(claw.getSensorPose(), Constants.CLAW_CLOSE_CUBE));
      }
      else {
        claw.set( clawPID.calculate(claw.getSensorPose(), Constants.CLAW_CLOSE_CONE));
      }
    }
    else {
      claw.set( clawPID.calculate(claw.getSensorPose(), Constants.CLAW_OPEN_CONE));
    }

    double armSpeed = armPID.calculate(arm.getSensorPose(), armTarget);

    if(armSpeed > armSpeedLimit) armSpeed = armSpeedLimit;
    else if(armSpeed < -armSpeedLimit) armSpeed = -armSpeedLimit;

    arm.set(armSpeed);

    // arm.set(Robot.m_robotContainer.manip.getLeftJoyY());
    SmartDashboard.putBoolean("claw toggle", clawToggle);
    SmartDashboard.putBoolean("cube mode", cubeMode);
    
    }

  public void setDoubleSubstation() {
    setArmTarget(Constants.ARM_SUBSTATION);
    substationMode = true;
  }


  public void toggleWrist() {
    wristToggle = !wristToggle;
  }

  public void toggleArm() {
    armToggle = !armToggle;
  }

  public void extendArm() {
    armTarget = Constants.ARM_SCORE;
    substationMode = false;
  }

  public void retractArm() {
    armTarget = Constants.ARM_CUBE_HOLD;
    substationMode = false;
  }

  public void armHoldPosition() {
    if(cubeMode) armTarget = Constants.ARM_CUBE_HOLD;
    else armTarget = Constants.ARM_CONE_HOLD;
    substationMode = false;
  }

  public void setArmTarget(double target) {
    armTarget = target;
    substationMode = false;
  }

  public void toggleCubeMode() {
    cubeMode = !cubeMode;
  }

  public void toggleClaw() {
    clawToggle = !clawToggle;
  }

  public void openClaw() {
    clawToggle = false;
  }
  
  public void closeClaw() {
    clawToggle = true;
  }

  public void wristFalconUp() {
    wristToggle = false;
  }

  public void wristFalconDown() {
    wristToggle = true;
  }

  public boolean getCubeMode() {
    return cubeMode;
  }

  public void setCubeMode() {
    cubeMode = true;
  }

  int sustain = 0;
  public void closeClawWhenSeen() {
    if (proximitySensor.getValue() > 1300 && proximitySensor.getValue() < 2500) {
      sustain++;
    }
    else {
      sustain = 0;
    }

    if(sustain >= 5) closeClaw();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("periodic wrist", wrist.getEncoder().getPosition());
    // SmartDashboard.putNumber("wrist pose", wrist.getEncoder().getPosition());
    SmartDashboard.putNumber("arm enc", arm.getSensorPose());
    SmartDashboard.putNumber("claw enc", claw.getSensorPose());
    // SmartDashboard.putNumber("distance sens", distanceSensor.getRange());

    SmartDashboard.putNumber("sensor deez", proximitySensor.getValue());
    SmartDashboard.putBoolean("does it work", proximitySensor.getValue() > 600 && proximitySensor.getValue() < 2000);
  }
}