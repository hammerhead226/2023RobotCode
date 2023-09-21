// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
public class OneCubeMidClearMobile extends SequentialCommandGroup {
  /** Creates a new OneCubeMidClearMobile. */
  public OneCubeMidClearMobile() {
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
      // do whatever here that strafes to the left
      // positive x goes left
      new RunCommand(() -> DriveTrain.getInstance().control(0, 0.275, 0), DriveTrain.getInstance()).withTimeout(4.25),
      // new RunCommand(() -> DriveTrain.getInstance().control(0,0, 0.275), DriveTrain.getInstance()).withTimeout(0.5),
      new InstantCommand(() -> DriveTrain.getInstance().control(0, 0, 0), DriveTrain.getInstance())
    );
  }
}
