// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ScoringStateManager;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Scoring extends SequentialCommandGroup {
  /** Creates a new Scoring. */
  public Scoring() {
    ScoringStateManager manager = new ScoringStateManager();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> manager.setArmTarget(Constants.ARM_SCORE)),
      new InstantCommand(manager::clawOuttake),
      new WaitCommand(1),
      new InstantCommand(() -> manager.setArmTarget(Constants.ARM_HOLD)),
      new Stow()
    );
  }
}
