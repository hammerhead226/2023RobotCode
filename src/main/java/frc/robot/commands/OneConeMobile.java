// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneConeMobile extends SequentialCommandGroup {
  /** Creates a new OneConeMobile. */
  public OneConeMobile() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      new InstantCommand(() -> Robot.m_robotContainer.elevator.setTarget(0), Robot.m_robotContainer.elevator),
      new WaitCommand(1.5),
      new InstantCommand(() -> Robot.m_robotContainer.gripper.extendArm(), Robot.m_robotContainer.gripper),
      new WaitCommand(1),
      new InstantCommand(() -> Robot.m_robotContainer.linearSlide.setTarget(47), Robot.m_robotContainer.linearSlide),
      new WaitCommand(3),
      new InstantCommand(() -> Robot.m_robotContainer.gripper.toggleClaw(), Robot.m_robotContainer.gripper),
      new WaitCommand(1),
      new InstantCommand(() -> Robot.m_robotContainer.linearSlide.setTarget(0), Robot.m_robotContainer.linearSlide),
      new WaitCommand(3),
      new InstantCommand(() -> Robot.m_robotContainer.gripper.retractArm(), Robot.m_robotContainer.gripper),
      new WaitCommand(0.5),
      new InstantCommand(() -> Robot.m_robotContainer.elevator.setTarget(1300), Robot.m_robotContainer.elevator),
      new WaitCommand(0.5),
      new InstantCommand(DriveTrain.getInstance()::reset, DriveTrain.getInstance()),
      new RunCommand(() -> DriveTrain.getInstance().toPose(new double[]{0, -144, 0}), DriveTrain.getInstance())




      
    );
  }
}
