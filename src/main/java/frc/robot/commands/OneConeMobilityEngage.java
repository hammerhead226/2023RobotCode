// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneConeMobilityEngage extends SequentialCommandGroup {
  /** Creates a new OneConeMobilityEngage. */
  public OneConeMobilityEngage() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitUntilCommand(Robot.m_robotContainer.manager::intakeTargetReached),
      new Level2(),
      new WaitUntilCommand(Robot.m_robotContainer.manager::linearSlideTargetReached),
      new WaitCommand(0.35),
      new Scoring(),
      new WaitCommand(0.5),
      new InstantCommand(() -> DriveTrain.getInstance().reset(), DriveTrain.getInstance()),
      // new InstantCommand(() -> DriveTrain.getInstance().toggleAuto(), DriveTrain.getInstance()),
      // new RunCommand(() -> DriveTrain.getInstance().control(-0.08, -0.7, 0), DriveTrain.getInstance()).until(DriveTrain.getInstance()::isChassisUnstable),
      // new RunCommand(() -> DriveTrain.getInstance().control(0.08, -0.85, 0), DriveTrain.getInstance()).until(DriveTrain.getInstance()::isChassisUnstable),
      new RunCommand(() -> DriveTrain.getInstance().control(0, 0.7, 0), DriveTrain.getInstance()).until(DriveTrain.getInstance()::isChassisUnstable),
      new RunCommand(() -> DriveTrain.getInstance().control(0, 0.35, 0), DriveTrain.getInstance()).until(DriveTrain.getInstance()::isChassisStable),
      new RunCommand(() -> DriveTrain.getInstance().control(0, 0.175, 0), DriveTrain.getInstance()).withTimeout(1.8),
      // new RunCommand(() -> DriveTrain.getInstance().control(0.04, -0.175, 0), DriveTrain.getInstance()).withTimeout(1.5),
      new InstantCommand(() -> DriveTrain.getInstance().control(0, 0, 0), DriveTrain.getInstance()),
      new WaitCommand(1),
      new RunCommand(() -> DriveTrain.getInstance().control(0, -0.7, 0), DriveTrain.getInstance()).until(DriveTrain.getInstance()::isChassisUnstable),
      new WaitCommand(0.1),
      new InstantCommand(() -> DriveTrain.getInstance().control(0, 0, 0), DriveTrain.getInstance()),
      new AutoBalance(false, 0.012)
      // new RunCommand(() -> DriveTrain.getInstance().control(0, 0.12, 0), DriveTrain.getInstance()).withTimeout(1),
      // new InstantCommand(() -> DriveTrain.getInstance().control(0, 0, 0), DriveTrain.getInstance())
    );
  }
}
