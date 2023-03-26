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
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneConeMobilityEngage extends SequentialCommandGroup {
  /** Creates a new OneConeMobilityEngage. */
  public OneConeMobilityEngage() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new WaitCommand(0.5),
      // new InstantCommand(() -> Robot.m_robotContainer.elevator.setTarget(-1100), Robot.m_robotContainer.lock)
      // .andThen(new WaitCommand(0.25))
      // .andThen(Robot.m_robotContainer.gripper::extendArm, Robot.m_robotContainer.lock)
      // .andThen(new WaitCommand(0.75))
      // .andThen(() -> Robot.m_robotContainer.linearSlide.setTarget(44), Robot.m_robotContainer.lock),
      // new WaitCommand(3),
      // new InstantCommand(() -> Robot.m_robotContainer.gripper.toggleClaw(), Robot.m_robotContainer.lock),
      // new WaitCommand(.5),
      // new InstantCommand(() -> Robot.m_robotContainer.linearSlide.setTarget(0), Robot.m_robotContainer.lock)
      // .andThen(new WaitCommand(1))
      // .andThen(() -> Robot.m_robotContainer.elevator.setTarget(800), Robot.m_robotContainer.lock)
      // .andThen(new WaitCommand(0.25))
      // .andThen(Robot.m_robotContainer.gripper::retractArm, Robot.m_robotContainer.lock),
      new InstantCommand(() -> Robot.m_robotContainer.elevator.setTarget(800), Robot.m_robotContainer.lock),
      new InstantCommand(() -> DriveTrain.getInstance().reset(), DriveTrain.getInstance()),
      new RunCommand(() -> DriveTrain.getInstance().control(0, -2.25, 0), DriveTrain.getInstance()).until(DriveTrain.getInstance()::isChassisUnstable),
      new InstantCommand(DriveTrain.getInstance()::toggleSpeed, DriveTrain.getInstance()),
      new InstantCommand(DriveTrain.getInstance()::reset, DriveTrain.getInstance()),
      // new InstantCommand(() -> DriveTrain.getInstance().toPose(new double[]{0, -50, 0}), DriveTrain.getInstance()),
      // new RunCommand(() -> DriveTrain.getInstance().toPose(new double[]{0, -50, 0}), DriveTrain.getInstance()).until(DriveTrain.getInstance()::atSetpoint),
      new RunCommand(() -> DriveTrain.getInstance().control(0, -1, 0), DriveTrain.getInstance()).until(DriveTrain.getInstance()::isChassisStable),
      new RunCommand(() -> DriveTrain.getInstance().control(0, -1, 0), DriveTrain.getInstance()).until(DriveTrain.getInstance()::isChassisStable),
      new WaitCommand(1),
      // new InstantCommand(() -> DriveTrain.getInstance().toPose(new double[]{0, -48, Math.PI}), DriveTrain.getInstance()),
      // new RunCommand(() -> DriveTrain.getInstance().toPose(new double[]{0, -48, Math.PI}), DriveTrain.getInstance()).until(DriveTrain.getInstance()::atSetpoint),
      new InstantCommand(DriveTrain.getInstance()::toggleSpeed, DriveTrain.getInstance()),
      new RunCommand(() -> DriveTrain.getInstance().control(0, 2, 0), DriveTrain.getInstance()).until(DriveTrain.getInstance()::isChassisUnstable),
      new AutoBalance(15)
    );
  }
}
