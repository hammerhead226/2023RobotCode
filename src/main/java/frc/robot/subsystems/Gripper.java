// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
// import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.wrappers.GenericMotor;
import frc.libs.wrappers.GenericMotor.PassiveMode;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Gripper extends SubsystemBase {
  private GenericMotor arm;
  private GenericMotor wheeledClaw; 

  private PIDController armPID;
  private DutyCycleEncoder armCoder;

  private CANSparkMax armSpark;

  private AbsoluteEncoder aCoder;


  private AnalogInput proximitySensor;

  private boolean armToggle = true;

  private double armTarget = Constants.ARM_STOW;


  private boolean cubeMode = false;

  private double armSpeedLimit;

  private boolean substationMode = false;

  int sustain = 0;


  public Gripper() {
    // wrist = new CANSparkMax(RobotMap.GRIPPER_WRIST, MotorType.kBrushless);
    wheeledClaw = new GenericMotor(new TalonFX(RobotMap.WHEELED_CLAW_MOTOR, Constants.CANBUS));
    arm = new GenericMotor(new TalonFX(RobotMap.ARM_MOTOR, Constants.CANBUS));
    // arm.setNeutralMode(PassiveMode.BRAKE);
    // armEncoder = new CANSparkMax(18, MotorType.kBrushed);

    armSpark = new CANSparkMax(18, MotorType.kBrushed);
    aCoder = armSpark.getAbsoluteEncoder(Type.kDutyCycle);

    aCoder.setPositionConversionFactor(8192);
    
    // armSpark.getEncoder(Type.kQuadrature, 8192).get

    wheeledClaw.inverted(true);


    // armCoder = new DutyCycleEncoder(18);
    // distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

    // wrist.setIdleMode(IdleMode.kBrake);
    wheeledClaw.setNeutralMode(PassiveMode.BRAKE);
    // arm.setNeutralMode(PassiveMode.BRAKE);

    armPID = new PIDController(Constants.ARM_GAINS[0], Constants.ARM_GAINS[1], Constants.ARM_GAINS[2]);
    armSpeedLimit = 0.65;
    this.proximitySensor = new AnalogInput(0);
  }

  public void run() {
    // if(substationMode) {
    //   stopClawWhenSeen();
    // }

    // wheeledClaw.set(0.3);
    // if(cubeMode) {
    //   wheeledClaw.set(wheeledClawPID.calculate(wheeledClaw.getSensorPose(), Constants.CLAW_CLOSE_CUBE));
    // }
    // else {
    //   wheeledClaw.set(wheeledClawPID.calculate(wheeledClaw.getSensorPose(), Constants.CLAW_CLOSE_CONE));
    // }
    
    
    double armSpeed = armPID.calculate(aCoder.getPosition(), armTarget);

    if(armSpeed > armSpeedLimit) armSpeed = armSpeedLimit;
    else if(armSpeed < -armSpeedLimit) armSpeed = -armSpeedLimit;

    arm.set(-armSpeed);

    // arm.set(Robot.m_robotContainer.manip.getLeftJoyY());
    // SmartDashboard.putBoolean("claw toggle", clawToggle);
    // SmartDashboard.putBoolean("cube mode", cubeMode);
    
    }
  
  public void setDoubleSubstation() {
    setArmTarget(Constants.ARM_SUBSTATION);
    substationMode = true;
  }

  public void setSubstationMode(boolean substationMode) {
    this.substationMode = substationMode;
  }

  public void wheeledClawIntake() {
    wheeledClaw.set(1);
  }

  public void wheeledClawOuttake() {
    wheeledClaw.set(-0.85);
  }

  public void wheeledClawStop() {
    wheeledClaw.set(0);
  }

  // public void toggleWrist() {
  //   wristToggle = !wristToggle;
  // }

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

  public double getArmTarget() {
    return armTarget;
  }

  public void toggleCubeMode() {
    cubeMode = !cubeMode;
  }

  // public void toggleClaw() {
  //   clawToggle = !clawToggle;
  // }

  // public void openClaw() {
  //   clawToggle = false;
  // }
  
  // public void closeClaw() {
  //   clawToggle = true;
  // }

  // public void wristFalconUp() {
  //   wristToggle = false;
  // }

  // public void wristFalconDown() {
  //   wristToggle = true;
  // }

  public boolean getCubeMode() {
    return cubeMode;
  }

  public void cubeModeOn() {
    cubeMode = true;
  }

  public void cubeModeOff() {
    cubeMode = false;
  }

  public boolean pieceDetected() {
    double distanceSensorVal = cubeMode ? Constants.CUBE_VALUE : Constants.CONE_VALUE;

    return proximitySensor.getValue() > distanceSensorVal && proximitySensor.getValue() < 2600;
  }
 
  
  public boolean stopClawWhenSeen() {
    double distanceSensorVal = cubeMode ? Constants.CUBE_VALUE : Constants.CONE_VALUE;

    if (proximitySensor.getValue() > distanceSensorVal && proximitySensor.getValue() < 2600) {
      sustain++;
    } else {
      sustain = 0;
    }

   
    if(sustain >= 3) {
      wheeledClaw.set(0); // if speed is too high sensor might not have enough time to react
      // if(!Robot.m_robotContainer.animation.isScheduled())
      // Robot.m_robotContainer.animation.schedule();
      sustain = 0;
      return true;
    }
    return false;
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gripper Sustain", sustain);
    SmartDashboard.putBoolean("Cube Mode", cubeMode);

    SmartDashboard.putNumber("Prox. Sensor", proximitySensor.getValue());
    SmartDashboard.putBoolean("Piece Detected?", pieceDetected());

    SmartDashboard.putNumber("Arm Enc", aCoder.getPosition());
  }
}

// 2249166 ur bals