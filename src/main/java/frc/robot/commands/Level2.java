// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LinearSlide;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Level2 extends SequentialCommandGroup {
  /** Creates a new Level3. */
  public Level2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(Robot.m_robotContainer.manager::setIntakeLower, Robot.m_robotContainer.lock),
      new WaitUntilCommand(Robot.m_robotContainer.manager::intakeTargetReached),
      new InstantCommand(() -> Robot.m_robotContainer.manager.setArmTarget(Gripper.getHold()), Robot.m_robotContainer.lock),
      new InstantCommand(() -> Robot.m_robotContainer.manager.setElevatorTarget(Elevator.getMid()), Robot.m_robotContainer.lock),
      new WaitUntilCommand(Robot.m_robotContainer.manager::armAndElevatorReached),
      new InstantCommand(() -> Robot.m_robotContainer.manager.setLinearSlideTarget(LinearSlide.getMid()), Robot.m_robotContainer.lock)
    );
  }
}