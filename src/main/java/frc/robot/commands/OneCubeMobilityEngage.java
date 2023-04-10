// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneCubeMobilityEngage extends SequentialCommandGroup {
  /** Creates a new OneConeMobilityEngage. */
  public OneCubeMobilityEngage() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(0.25),
      new InstantCommand(Robot.m_robotContainer.gripper::setCubeMode, Robot.m_robotContainer.lock),
      new InstantCommand(() -> Robot.m_robotContainer.elevator.setTarget(Constants.ELEVATOR_HIGH), Robot.m_robotContainer.lock)
          .andThen(new WaitCommand(0.1))
          .andThen(() -> Robot.m_robotContainer.gripper.armHoldPosition(), Robot.m_robotContainer.lock)
          .andThen(new WaitCommand(0.25))
          .andThen(() -> Robot.m_robotContainer.gripper.setArmTarget(Constants.ARM_SCORE), Robot.m_robotContainer.lock)
          .andThen(new WaitCommand(0.75))
          .andThen(() -> Robot.m_robotContainer.linearSlide.setTarget(Constants.LS_HIGH), Robot.m_robotContainer.lock),
      new WaitCommand(1.5),
      new InstantCommand(Robot.m_robotContainer.gripper::openClaw, Robot.m_robotContainer.lock),
      new WaitCommand(0.5),
      new InstantCommand(() -> Robot.m_robotContainer.linearSlide.setTarget(Constants.LS_RETRACTED), Robot.m_robotContainer.lock)
      .andThen(new WaitCommand(0.5))
      .andThen(Robot.m_robotContainer.gripper::armHoldPosition, Robot.m_robotContainer.lock)
      .andThen(Robot.m_robotContainer.gripper::closeClaw, Robot.m_robotContainer.lock)
      .andThen(new WaitCommand(0.25)),
      new InstantCommand(() -> Robot.m_robotContainer.elevator.setTarget(800), Robot.m_robotContainer.lock),
      new InstantCommand(() -> DriveTrain.getInstance().reset(), DriveTrain.getInstance()),
      new RunCommand(() -> DriveTrain.getInstance().control(0, -2.25, 0), DriveTrain.getInstance()).until(DriveTrain.getInstance()::isChassisUnstable),
      new RunCommand(() -> DriveTrain.getInstance().control(0, -0.5, 0), DriveTrain.getInstance()).until(DriveTrain.getInstance()::isChassisStable),
      new RunCommand(() -> DriveTrain.getInstance().control(0, -0.25, 0), DriveTrain.getInstance()).withTimeout(1.5),
      new InstantCommand(() -> DriveTrain.getInstance().control(0, 0, 0), DriveTrain.getInstance()),
      new WaitCommand(1),
      new RunCommand(() -> DriveTrain.getInstance().control(0, 2.25, 0), DriveTrain.getInstance()).until(DriveTrain.getInstance()::isChassisUnstable),
      new WaitCommand(0.1),
      new InstantCommand(() -> DriveTrain.getInstance().control(0, 0, 0), DriveTrain.getInstance()),
      new AutoBalance(false, 0.0175)
    );
  }
}
