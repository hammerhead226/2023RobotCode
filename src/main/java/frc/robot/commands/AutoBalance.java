// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  PIDController con = DriveTrain.getInstance().getBalanceController();
  double gyroThresh;
  boolean direction;
  public AutoBalance(boolean direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveTrain.getInstance());
    this.direction = direction;
  }

  public AutoBalance(boolean direction, double p) {
    addRequirements(DriveTrain.getInstance());
    this.direction = direction;
    con.setP(p);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("auto balance running", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveTrain.getInstance().control(0, con.calculate(DriveTrain.getInstance().getGyroPitch()), 0); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveTrain.getInstance().control(0, 0, 0.001);
  }

  // Returns true when the command should end.
  int sustain = 0;
  @Override
  public boolean isFinished() {
    // if(Math.abs(DriveTrain.getInstance().getGyroTilt()) < 4.5) sustain++;
    // else sustain = 0;
    // return sustain >= 50;
    // return Math.abs(DriveTrain.getInstance().getGyroTilt()) < gyroThresh;
    // return DriveTrain.getInstance().getGyroTilt() < 15 || DriveTrain.getInstance().getGyroTilt() > -2;
    if(direction) return DriveTrain.getInstance().getGyroPitch() < 15;
    else return DriveTrain.getInstance().getGyroPitch() > -3;
  }
}
