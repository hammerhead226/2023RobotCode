// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.libs.wrappers.LimeLight;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class LimelightLineUp extends CommandBase {
  /** Creates a new LimelightLineUp. */
  // private boolean blueAlliance;
  private double gyroYaw;
  private double velocityMultiplier;

  public LimelightLineUp() {
    addRequirements(DriveTrain.getInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // public LimelightLineUp(boolean blueAlliance) {
  //   addRequirements(DriveTrain.getInstance());
  //   // this.blueAlliance = blueAlliance;
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("limelight started", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("moving?????!?!??!", true);
    if (LimeLight.getValue() == 1) {
      double translationSpeed;
      double rotateSpeed;

      velocityMultiplier = gyroYaw < 0 ? -1 : 1;
      
      //TODO:: GIVE THESE THRESHOLDS ACTUAL VALUES LOL
      if (Math.abs(2 - LimeLight.getHorizontalOffset()) >= Constants.TRANSLATION_CUTOFF_THRESH) {
        translationSpeed = DriveTrain.getInstance().getLimelightController().calculate(LimeLight.getHorizontalOffset(), 2);
      } else {
        translationSpeed = 0;
      }

      if (Math.PI - Math.abs(gyroYaw % (2 * Math.PI)) >= Constants.ROTATE_CUTOFF_THRESH) {
        rotateSpeed = -DriveTrain.getInstance().getRotateController().calculate(Math.abs(gyroYaw % (2 * Math.PI)), Math.PI) * velocityMultiplier;
      } else {
        rotateSpeed = 0;
      }
      
      gyroYaw = DriveTrain.getInstance().getGyroYaw();
      
      DriveTrain.getInstance().control(translationSpeed, Robot.m_robotContainer.driver.getLeftJoyY(), rotateSpeed);
      // TODO:: may need to change later but field element at woodshop is aligned with tx is 3.5
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (Math.abs(LimeLight.getHorizontalOffset()) <= Constants.LIMELIGHT_THRESH) {
    //   SmartDashboard.putBoolean("dobne?????!?!??!", true);
    // } else {
    //   SmartDashboard.putBoolean("done?????!?!??!", false);
    // }
    // the limelight is lining up before the gyro can correct itself ?

    return Math.abs(LimeLight.getHorizontalOffset()) <= Constants.LIMELIGHT_THRESH && Math.abs(gyroYaw % (2 * Math.PI)) <= Constants.ROTATE_THRESH;
    // return false;
  }
}
