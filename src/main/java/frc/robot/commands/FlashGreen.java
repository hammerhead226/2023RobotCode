// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LED;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FlashGreen extends SequentialCommandGroup {
  /** Creates a new FlashGreen. */
  public FlashGreen(double delay, int iterations, LED led) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double number = 0.75;
    double interval = delay;
    int iter = iterations;
    SequentialCommandGroup[] cg = new SequentialCommandGroup[iter];
    for(int i = 0; i < iter; i++) {
      cg[i] = new SequentialCommandGroup(
        new InstantCommand(() -> RobotContainer.led.setColor(number), led),
        new WaitCommand(interval),
        new InstantCommand(() -> RobotContainer.led.setColor(0.99), led),
        new WaitCommand(interval)
      );
    }

    addCommands(
      cg      
    );
  }
}
