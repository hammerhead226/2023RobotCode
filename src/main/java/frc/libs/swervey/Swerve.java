// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.libs.swervey;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.libs.wrappers.GenericEncoder;
import frc.libs.wrappers.GenericMotor;
import frc.libs.wrappers.Gyro;
import frc.libs.wrappers.PIDController;

/**
 * <p> Handles the Swerve DriveTrain </p>
 * @author Anish Chandra
 * @apiNote Use the SwerveBuilder class to construct Swerve
*/
public class Swerve {

  private SwerveModule[] modules;
  private Gyro gyro;

  private PIDController driveController;
  private PIDController steerController;
  private PIDController rotateController;

  private double[] speeds;
  private double[] thetas;

  private double lowPercentSpeed, topPercentSpeed, currentPercentSpeed;
  private double accelerationRate, initialSpeed;
  private boolean highSpeedMode;
  private double[] rotationAngles;

  private double rotateGainsThreshold;
  private double rotateVelocityThreshold;
  private double[] rotateHighGains;
  private double[] rotateLowGains;
  private boolean isGyroAngleSet;
  private double gyroHold;
  private double lastHeading;

  private double x, y;
  private double lastX, lastY;
  private double ticksPerFeet;
  private double[] target;
  private double allowedTranslationalError;
  private double allowedRotationalError;

  private boolean isFieldCentric;


  public Swerve(GenericMotor[] drives, GenericMotor[] steers, GenericEncoder[] encoders, Gyro gyro, double[][] modulePositions, int numberOfModules) {
    modules = new SwerveModule[numberOfModules];
    speeds = new double[numberOfModules];
    thetas = new double[numberOfModules];
    this.rotationAngles = new double[numberOfModules];
    this.driveController = new PIDController(0, 0, 0);
    this.steerController = new PIDController(0, 0, 0);
    this.rotateController = new PIDController(0, 0, 0);
    this.highSpeedMode = false;
    this.isFieldCentric = true;

    this.gyro = gyro;
    this.x = 0;
    this.y = 0;
    this.lastX = 0;
    this.lastY = 0;
    this.target = new double[3];

    this.lastHeading = gyro.getYaw();

    for(int i = 0; i < numberOfModules; i++) {
        modules[i] = new SwerveModule(drives[i],
                                      steers[i],
                                      encoders[i],
                                      steerController,
                                      modulePositions[i]);
        speeds[i] = 0;
        thetas[i] = 0;
    }

    for(int i = 0; i < rotationAngles.length; i++) 
      rotationAngles[i] = Math.atan2(modulePositions[i][1], modulePositions[i][0]) + (Math.PI/2);
    
    reset();
    zeroGyro();
  
   }

  public void configureSteerPIDGains(double[] highGains, double[] lowGains) {
    for(SwerveModule mod : modules) mod.configureSteerPIDGains(highGains, lowGains);
  }

  public void configureDrivePIDGains(double[] gains) {
    this.driveController.setPID(gains);
  }

  public void configureRotatePIDGains(double[] highGains, double[] lowGains) {
    this.rotateHighGains = highGains;
    this.rotateLowGains = lowGains;
    this.rotateController.setPID(rotateHighGains);
  }

  public void configureThresholds(double sThresh, double rThresh, double rVelThresh) {
    for(SwerveModule mod : modules) mod.configureSteerThreshold(sThresh);
    this.rotateGainsThreshold = rThresh;
    this.rotateVelocityThreshold = rVelThresh;
  }

  public void configureAutonomousParameters(double ticksPerFeet, double translationalError, double rotationalError, double velocityFeedForwardGain) {
    this.ticksPerFeet = ticksPerFeet;
    this.allowedTranslationalError = translationalError;
    this.allowedRotationalError = rotationalError;
  }

  public void configureSpeeds(double topSpeed, double lowSpeed) {
    this.topPercentSpeed = topSpeed;
    this.lowPercentSpeed = lowSpeed;
    this.currentPercentSpeed = this.lowPercentSpeed * 0.3;
  }

  public void configureAccelerationParameters(double accelerationRate, double initialSpeed) {
    this.accelerationRate = accelerationRate;
    this.initialSpeed = initialSpeed;
  }

  public void control(double x, double y, double rotate) {
    //Controlled Acceleration
    if(highSpeedMode) {
      if(x == 0 && y == 0 && rotate == 0) {
        currentPercentSpeed = topPercentSpeed * initialSpeed;
      }
      else if(currentPercentSpeed < topPercentSpeed) {
        currentPercentSpeed += accelerationRate;
      }
      
    }
    else {
      if(x == 0 && y == 0 && rotate == 0) {
        currentPercentSpeed = lowPercentSpeed * initialSpeed;
      }
      else if(currentPercentSpeed < lowPercentSpeed) {
        currentPercentSpeed += accelerationRate;
      }
    }
    
    if(rotate == 0 && isFieldCentric) {
      if(!isGyroAngleSet) {
        gyroHold = gyro.getYaw();
        isGyroAngleSet = true;
      }

      if(Math.hypot(x, y) < rotateGainsThreshold) rotateController.setPID(rotateHighGains);
      else rotateController.setPID(rotateLowGains);

      rotate = rotateController.calculate(gyro.getYaw(), gyroHold);
      if(Math.abs(rotate) < rotateVelocityThreshold) rotate = 0;
    }
    else {
      isGyroAngleSet = false;
    }

    double gyroYaw = isFieldCentric ? gyro.getYaw() : 0;

    for(int i = 0; i < modules.length; i++) {

      double rotateVectorX = rotate * Math.cos(rotationAngles[i] + gyroYaw);
      double rotateVectorY = rotate * Math.sin(rotationAngles[i] + gyroYaw);

      double targetVectorX = x + rotateVectorX;
      double targetVectorY = y + rotateVectorY;

      double theta = Math.atan2(targetVectorY, targetVectorX);
      double speed = Math.hypot(targetVectorY, targetVectorX);

      theta -= gyroYaw;

      if(!(x == 0 && y == 0 && rotate == 0)) {
        thetas[i] = theta;
      }
      speeds[i] = speed;
    }
    speeds = normalize(speeds);

    for(int i = 0; i < modules.length; i++) {
      modules[i].set(speeds[i] * currentPercentSpeed, thetas[i], gyroYaw);
    }
  }

  private double[] normalize(double[] arr) {
    double maxVal = 1;
    for(int i = 0; i < arr.length; i++) {
      if(arr[i] > maxVal) {
        maxVal = arr[i];
      }
    }

    for(int i = 0; i < arr.length; i++) {
      arr[i] /= maxVal;
    }

    return arr;
  }

  public void toggleSpeed() {
    if(highSpeedMode) {
      currentPercentSpeed = lowPercentSpeed * initialSpeed;
    }
    else {
      currentPercentSpeed = topPercentSpeed * initialSpeed;
    }
    highSpeedMode = !highSpeedMode;
  }

  public void enableRobotCentric() {
    this.isFieldCentric = false;
  }

  public void enableFieldCentric() {
    this.isFieldCentric = true;
  }

  public boolean isFieldCentric() {
    return isFieldCentric;
  }

  public void setTargetPosition(double[] target) {
    this.target = target;
  }

  public double getCurrentSpeedMultiplier() {
    return currentPercentSpeed;
  }

  public double getModuleRotationalPose(int module) {
    return modules[module].getModuleRotationalPose();
  }

  public double[] getModulePose(int i) {
    return new double[] {modules[i].getCurrentModulePosition()[0]/ticksPerFeet, modules[i].getCurrentModulePosition()[1]/ticksPerFeet};
  }

  public double getModuleDrivePose(int i) {
    return modules[i].getDrivePose();
  }

  public double[][] getAllModuleDrivePoseDelta() {
    double[][] out = new double[modules.length][2];

    for(int i = 0; i < modules.length; i++) {
      double distance = modules[i].getDrivePoseDelta();
      double angle = modules[i].getSteerAngleDelta();

      out[i] = new double[]{distance, angle};
    }

    return out;
  }

  public double[][] getAllModuleDrivePose() {
    double[][] out = new double[modules.length][2];

    for(int i = 0; i < modules.length; i++) {
      double distance = modules[i].getDrivePose();
      double angle = modules[i].getModuleRotationalPose();

      out[i] = new double[]{distance, angle};
    }
    
    return out;
  }

  public double getModuleDriveVelocity(int i) {
    return modules[i].getDriveVelocity();
  }

  public double getModuleSteerVelocity(int i) {
    return modules[i].getSteerVelocity();
  }

  public double[] getPose() {
    x = 0;
    y = 0;
    for(int i = 0; i < modules.length; i++) {
      x += modules[i].getCurrentModulePosition()[0]/modules.length;
      y += modules[i].getCurrentModulePosition()[1]/modules.length;
    }
    double deltaX = x - lastX;
    double deltaY = y - lastY;

    lastX = x;
    lastY = y;
    return new double[]{x/ticksPerFeet, y/ticksPerFeet, gyro.getYaw()};
  }

  public double getHeading() {
    return gyro.getYaw();
  }

  public double getHeadingDelta() {
    double out = gyro.getYaw() - lastHeading;
    lastHeading = gyro.getYaw();
    return out;
  }
  
  public void toPose(double[] target) {
    double[] currentPose = getPose();
    this.target = target;
    double xErr = driveController.calculate(currentPose[0], target[0]);
    double yErr = driveController.calculate(currentPose[1], target[1]);
    double speed = Math.hypot(xErr, yErr);

    if(Math.abs(target[0] - currentPose[0]) < allowedTranslationalError) xErr = 0;
    if(Math.abs(target[1] - currentPose[1]) < allowedTranslationalError) yErr = 0;

    if(Math.abs(speed) < rotateGainsThreshold) rotateController.setPID(rotateHighGains);
    else rotateController.setPID(rotateLowGains);

    SmartDashboard.putBoolean("rotate high", Math.abs(speed) < rotateGainsThreshold);


    double rotateErr = rotateController.calculate(currentPose[2], target[2]);
    if(Math.abs(target[2] - currentPose[2]) < allowedRotationalError) rotateErr = 0;

    SmartDashboard.putNumber("xErr", xErr);
    SmartDashboard.putNumber("yErr", yErr);
    SmartDashboard.putNumber("rotateErr", rotateErr);

    SmartDashboard.putNumber("target[0]", target[0]);
    SmartDashboard.putNumber("target[1]", target[1]);
    SmartDashboard.putNumber("target[2]", target[2]);

    control(xErr, yErr, rotateErr);
  }

  public boolean atSetpoint() {
    double[] currentPose = getPose();
    double xErr = target[0] - currentPose[0];
    double yErr = target[1] - currentPose[1];
    double thetaErr = target[2] - currentPose[2];

    return (Math.abs(xErr) <= allowedTranslationalError) && (Math.abs(yErr) <= allowedTranslationalError) && (Math.abs(thetaErr) <= allowedRotationalError);
  }

  public void zeroGyro() {
    gyro.zeroGyro();
    gyroHold = 0;
    isGyroAngleSet = true;
  }

  public void reset() {
    for(int i = 0; i < modules.length; i++) {
      modules[i].reset();
    }
  }
}