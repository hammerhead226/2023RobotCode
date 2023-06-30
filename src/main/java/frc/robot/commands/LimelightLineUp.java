// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Mat;

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
      SmartDashboard.putBoolean("does it reach?????!?!??!", true);
      // if (blueAlliance) {
      //   velocityMultiplier *= -1;
      // } 
      gyroYaw = DriveTrain.getInstance().getGyroYaw();
      velocityMultiplier = gyroYaw < 0 ? -1 : 1;
      DriveTrain.getInstance().control
      // TODO:: may need to change later but field element at woodshop is aligned with tx is 3.5
      (DriveTrain.getInstance().getLimelightController().calculate(LimeLight.getHorizontalOffset(), 2), 
      Robot.m_robotContainer.driver.getLeftJoyY(), 
      -DriveTrain.getInstance().getRotateController().calculate(Math.abs(gyroYaw % (2 * Math.PI)), Math.PI) * velocityMultiplier);
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

    // this probably isnt going to work lols
    Robot.m_robotContainer.driver.vibrate();
    return Math.abs(LimeLight.getHorizontalOffset()) <= Constants.LIMELIGHT_THRESH && Math.abs(gyroYaw % (2 * Math.PI)) <= Constants.ROTATE_THRESH;
    // return false;
  }
}
