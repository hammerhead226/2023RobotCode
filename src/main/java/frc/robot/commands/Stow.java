// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ScoringStateManager;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Stow extends SequentialCommandGroup {
  /** Creates a new Stow. */
  public Stow() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> Robot.m_robotContainer.manager.setLinearSlideTarget(Constants.LS_RETRACTED), Robot.m_robotContainer.lock),
      new WaitUntilCommand(Robot.m_robotContainer.manager::linearSlideTargetReached),
      new InstantCommand(() -> Robot.m_robotContainer.manager.vibrate(Robot.m_robotContainer.driver), Robot.m_robotContainer.lock),
      new InstantCommand(Robot.m_robotContainer.manager::setIntakeLower, Robot.m_robotContainer.lock),
      new WaitUntilCommand(Robot.m_robotContainer.manager::intakeTargetReached),
      new InstantCommand(() -> Robot.m_robotContainer.manager.setElevatorTarget(Constants.ELEVATOR_HOLD), Robot.m_robotContainer.lock),
      new InstantCommand(() -> Robot.m_robotContainer.manager.setArmTarget(Constants.ARM_STOW), Robot.m_robotContainer.lock)
    );
  }
}
