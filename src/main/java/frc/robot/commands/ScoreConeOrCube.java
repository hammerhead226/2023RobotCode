// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeOrCube extends SequentialCommandGroup {
  /** Creates a new ScoreConeOrCube. */
  public ScoreConeOrCube() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitUntilCommand(Robot.m_robotContainer.manager::intakeTargetReached),
      new Level3(),
      new WaitUntilCommand(Robot.m_robotContainer.manager::linearSlideTargetReached),
      new WaitCommand(0.35),
      new Scoring()
    );
  }
}
