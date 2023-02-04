// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.revrobotics.RelativeEncoder;
// import com.revrobotics.Rev2mDistanceSensor;
//import com.revrobotics.SparkMaxPIDController; COULD POTENTIALLY USE THIS
// import com.revrobotics.Rev2mDistanceSensor.Port;
// import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
// import com.revrobotics.Rev2mDistanceSensor.Unit;

import frc.libs.wrappers.GenericMotor;
import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;

public class LinearSlide extends SubsystemBase {
  /** Creates a new LinearSlide. */
  private GenericMotor slider;
  private TalonFX sliderTalon;
  // private Rev2mDistanceSensor distanceSensor;
  private double initialPos;
  private boolean manual;
  //PIDController pid;
  private PIDController pid;
  public LinearSlide() {
      slider = new GenericMotor(sliderTalon);
      // distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
      // distanceSensor.setDistanceUnits(Unit.kInches);
      // distanceSensor.setRangeProfile(RangeProfile.kHighAccuracy);
      // initialPos = distanceSensor.getRange();
      
      pid = new PIDController(Constants.LINEAR_SLIDE_GAINS[0], Constants.LINEAR_SLIDE_GAINS[1], Constants.LINEAR_SLIDE_GAINS[2]);
  }

  public double getDistance() {
      return (slider.getSensorPose());
  }

  public void toggleManual(){
      manual = !manual;
  }

  public void extendDistanceLow() {
    slider.set(pid.calculate(getDistance(), Constants.EXTEND_LOW));
  }
  public void extendDistanceMid() {
    slider.set(pid.calculate(getDistance(), Constants.EXTEND_MID));
  }
  public void extendDistanceHigh() {
    slider.set(pid.calculate(getDistance(), Constants.EXTEND_HIGH));
  }

  public void retractSlider() {
    slider.set(pid.calculate(getDistance(), initialPos));
  }

  public void runManual(double speed)
  {
    if(manual){
      if(!(getDistance() <= Constants.SLIDE_MIN_POSITION || getDistance() >= Constants.SLIDE_MAX_POSITION))
        slider.set(speed * Constants.LINEAR_SLIDE_COEFFICIENT);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
