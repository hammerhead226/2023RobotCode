// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Handoff extends SequentialCommandGroup {
  /** Creates a new Handoff. */
  public Handoff() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> Robot.m_robotContainer.manager.setCubeMode(true), Robot.m_robotContainer.lock),
      // new InstantCommand(() -> Robot.m_robotContainer.manager.setIntakeHigh(false), Robot.m_robotContainer.lock),
      // new WaitUntilCommand(Robot.m_robotContainer.manager::intakeTargetReached),
      // new InstantCommand(() -> Robot.m_robotContainer.manager.setArmTarget(Constants.ARM_HANDOFF), Robot.m_robotContainer.lock),
      // new WaitUntilCommand(Robot.m_robotContainer.manager::armTargetReached),
      new InstantCommand(Robot.m_robotContainer.manager::runIn, Robot.m_robotContainer.lock),
      new InstantCommand(Robot.m_robotContainer.manager::clawIntake, Robot.m_robotContainer.lock),
      new WaitUntilCommand(Robot.m_robotContainer.manager::pieceDetected),
      new InstantCommand(Robot.m_robotContainer.manager::stopClaw, Robot.m_robotContainer.lock),
      new InstantCommand(Robot.m_robotContainer.manager::stopIntake, Robot.m_robotContainer.lock),
      new InstantCommand(() -> Robot.m_robotContainer.manager.setCubeMode(false), Robot.m_robotContainer.lock)

    );
  }
}
