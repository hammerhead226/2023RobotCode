// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.libs.swerveyshark.sharkexe.SharkExecutor;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.balls;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceAutons extends SequentialCommandGroup {
  /** Creates a new TestNewAuto. */
  private double startTime;

  public ThreePieceAutons(String pathName) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Consumer<double[]> toPose = (target) -> DriveTrain.getInstance().toPose(target);
    
    addCommands(
      new InstantCommand(DriveTrain.getInstance()::reset, DriveTrain.getInstance()),
      new InstantCommand(Robot.m_robotContainer.intake::runOut, Robot.m_robotContainer.lock),
      new Stow(),
      // new InstantCommand(() -> {
      //   try {
      //     SharkExecutor.loadAndConfigurePath(path, toPose);
      //   } catch (IOException e) {
      //     // TODO Auto-generated catch block
      //     e.printStackTrace();
      //   }
      // }, DriveTrain.getInstance()),
      new InstantCommand(Robot.m_robotContainer.intake::stop, Robot.m_robotContainer.lock),
      new InstantCommand(this::setStartTime, Robot.m_robotContainer.lock).alongWith(new RunCommand(()-> SharkExecutor.executeNextAvailableStep(pathName, System.currentTimeMillis()/1000. - startTime), DriveTrain.getInstance()).until(() -> SharkExecutor.isFinished(pathName)))
    );
  }

  public void setStartTime() {
    startTime = System.currentTimeMillis()/1000.;
  }
}
