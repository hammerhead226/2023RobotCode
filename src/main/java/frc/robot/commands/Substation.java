// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ScoringStateManager;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Substation extends SequentialCommandGroup {
  /** Creates a new Substation. */
  public Substation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(Robot.m_robotContainer.manager::setIntakeLower, Robot.m_robotContainer.lock),
      new WaitUntilCommand(Robot.m_robotContainer.manager::intakeTargetReached),
      new InstantCommand(() -> Robot.m_robotContainer.manager.setElevatorTarget(Constants.ELEVATOR_SUBSTATION)),
      new InstantCommand(() -> Robot.m_robotContainer.manager.setArmTarget(Constants.ARM_SUBSTATION)),
      new WaitUntilCommand(Robot.m_robotContainer.manager::armAndElevatorReached),
      new InstantCommand(() -> Robot.m_robotContainer.manager.setIntakeHigh(true), Robot.m_robotContainer.lock),
      new InstantCommand(() -> Robot.m_robotContainer.manager.setLinearSlideTarget(Constants.LS_SUBSTATION)),
      new InstantCommand(Robot.m_robotContainer.manager::clawIntake),
      new WaitUntilCommand(Robot.m_robotContainer.manager::stopClawWhenSeen),
      new Stow(),
      new InstantCommand(() -> Robot.m_robotContainer.manager.setIntakeHigh(true), Robot.m_robotContainer.lock)
    );
  }
}
