// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueOneConeMobile extends SequentialCommandGroup {
  /** Creates a new OneConeMobile. */
  public BlueOneConeMobile() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new Level3(),
      new WaitUntilCommand(Robot.m_robotContainer.manager::linearSlideTargetReached),
      new WaitCommand(0.25),
      new Scoring(),
      new WaitCommand(0.5),
      new InstantCommand(DriveTrain.getInstance()::reset, DriveTrain.getInstance()),
      new InstantCommand(() -> DriveTrain.getInstance().toPose(new double[] {0, -150, 0}), DriveTrain.getInstance()),
      new RunCommand(() -> DriveTrain.getInstance().toPose(new double[]{0, -150, 0}), DriveTrain.getInstance()).until(DriveTrain.getInstance()::atSetpoint),
      new InstantCommand(() -> DriveTrain.getInstance().toPose(new double[]{0, -150, -Math.PI}), DriveTrain.getInstance()),
      new RunCommand(() -> DriveTrain.getInstance().toPose(new double[]{0, -150, -Math.PI}), DriveTrain.getInstance()).until(DriveTrain.getInstance()::atSetpoint)




      
    );
  }
}
