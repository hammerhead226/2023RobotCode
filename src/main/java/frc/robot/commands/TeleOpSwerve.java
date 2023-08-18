// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TeleOpSwerve extends CommandBase {
  private Swerve swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;

  public TeleOpSwerve(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;

    addRequirements(swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationalVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.CONTROLLER_DEADBAND);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.CONTROLLER_DEADBAND);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.CONTROLLER_DEADBAND);

    // swerve.control(
    //   new Translation2d(translationalVal, strafeVal).times(5),
    //   rotationVal * Math.PI,
    //   !robotCentricSup.getAsBoolean(),
    //    true);

    swerve.control(
    new Translation2d(translationalVal, strafeVal).times(5),
    rotationVal * Math.PI,
    true,
      true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
