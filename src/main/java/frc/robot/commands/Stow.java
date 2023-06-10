// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ScoringStateManager;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Stow extends SequentialCommandGroup {
  /** Creates a new Stow. */
  public Stow() {
    ScoringStateManager manager = new ScoringStateManager();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> manager.setLinearSlideTarget(0)),
      new WaitUntilCommand(manager::linearSlideTargetReached),
      new InstantCommand(() -> manager.setElevatorTarget(Constants.ELEVATOR_STOW)),
      new InstantCommand(() -> manager.setArmTarget(Constants.ARM_STOW))
    );
  }
}
