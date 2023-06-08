// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.xml.namespace.QName;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.libs.swervey.MotionOfTheOcean;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  public TestAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new WaitCommand(0.25),
      // new InstantCommand(() -> Robot.m_robotContainer.elevator.setTarget(Constants.ELEVATOR_HIGH), Robot.m_robotContainer.lock)
      //     .andThen(new WaitCommand(0.1))
      //     .andThen(() -> Robot.m_robotContainer.gripper.armHoldPosition(), Robot.m_robotContainer.lock)
      //     .andThen(new WaitCommand(0.25))
      //     .andThen(() -> Robot.m_robotContainer.gripper.setArmTarget(Constants.ARM_SCORE), Robot.m_robotContainer.lock)
      //     .andThen(new WaitCommand(0.75))
      //     .andThen(() -> Robot.m_robotContainer.linearSlide.setTarget(Constants.LS_HIGH), Robot.m_robotContainer.lock),
      // new WaitCommand(1.5),
      // new InstantCommand(Robot.m_robotContainer.gripper::openClaw, Robot.m_robotContainer.lock),
      // new WaitCommand(0.5),
      // new InstantCommand(() -> Robot.m_robotContainer.linearSlide.setTarget(Constants.LS_RETRACTED), Robot.m_robotContainer.lock)
      // .andThen(new WaitCommand(0.5))
      // .andThen(Robot.m_robotContainer.gripper::armHoldPosition, Robot.m_robotContainer.lock)
      // .andThen(Robot.m_robotContainer.gripper::closeClaw, Robot.m_robotContainer.lock)
      // .andThen(new WaitCommand(0.25)),
      new InstantCommand(() -> Robot.m_robotContainer.elevator.setTarget(500), Robot.m_robotContainer.lock),
      new InstantCommand(() -> DriveTrain.getInstance().reset()),
      new InstantCommand(() -> MotionOfTheOcean.Executor.resetExecutor(DriveTrain.getInstance()::reset)),
      new InstantCommand(()-> MotionOfTheOcean.Executor.loadRecordings("/paths/red_export_path.csv")),
      new InstantCommand(() -> MotionOfTheOcean.Executor.selectRecording("/paths/red_export_path.csv")),
      new InstantCommand(() -> DriveTrain.getInstance().reset()),
      new InstantCommand(()-> DriveTrain.getInstance().togglePlayback()),
      new WaitCommand(0.5),
      new InstantCommand(() -> Robot.m_robotContainer.linearSlide.setTarget(Constants.LS_RETRACTED), Robot.m_robotContainer.lock)
      .andThen(Robot.m_robotContainer.gripper::cubeModeOn, Robot.m_robotContainer.lock)
      .andThen(new WaitCommand(0.5))
      .andThen(Robot.m_robotContainer.gripper::armHoldPosition, Robot.m_robotContainer.lock)
      .andThen(new WaitCommand(0.25))
      .andThen(() -> Robot.m_robotContainer.elevator.setTarget(Constants.ELEVATOR_INTAKE), Robot.m_robotContainer.lock)
      // .andThen(Robot.m_robotContainer.gripper::openClaw, Robot.m_robotContainer.lock)
      .andThen(new WaitCommand(0.5))
      .andThen(() -> Robot.m_robotContainer.gripper.setArmTarget(Constants.ARM_INTAKE), Robot.m_robotContainer.lock),
      new WaitCommand(0.25),
      new InstantCommand(Robot.m_robotContainer.intake::extendIntake, Robot.m_robotContainer.lock),
      new InstantCommand(Robot.m_robotContainer.intake::runIn, Robot.m_robotContainer.lock),
      new WaitCommand(1.5),
      new InstantCommand(Robot.m_robotContainer.intake::stop, Robot.m_robotContainer.lock),
      new WaitCommand(1.5),
      new InstantCommand(Robot.m_robotContainer.intake::runOut, Robot.m_robotContainer.lock),
      new WaitCommand(2.5),
      new InstantCommand(Robot.m_robotContainer.intake::stop, Robot.m_robotContainer.lock)
    );
  }
}
