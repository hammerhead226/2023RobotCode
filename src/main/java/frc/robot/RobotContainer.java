// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.Elevator;
import frc.libs.wrappers.Controller;
import frc.robot.subsystems.ActiveFloor;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final Controller driver = new Controller(0, Constants.CONTROLLER_DEADBAND);
  public static final Controller manip = new Controller(0, Constants.CONTROLLER_DEADBAND);
  
  public static final Elevator elevator = new Elevator();
  public static final Gripper gripper = new Gripper();
  public static final ActiveFloor activeFloor = new ActiveFloor();
  public static final Intake intake = new Intake();
  public static final LinearSlide linearSlide = new LinearSlide();
  public static final LED led = new LED();
  public static final Vision vision = new Vision();
  
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // gripper.setDefaultCommand(new RunCommand(gripper::run, gripper));
    // intake.setDefaultCommand(new RunCommand(intake::run, intake));
    linearSlide.setDefaultCommand(new RunCommand(linearSlide::run, linearSlide));
  }
  
  private void configureBindings() {
    // manip.getAButton().onTrue(new InstantCommand(gripper::toggleClaw, gripper));
    // manip.getXButton().onTrue(new InstantCommand(gripper::toggleWrist, gripper));
    // manip.getYButton().onTrue(new InstantCommand(gripper::toggleArm, gripper));

    manip.getAButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(50), linearSlide));
    manip.getBButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(1000), linearSlide));
    manip.getYButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(2000), linearSlide));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand(() -> System.out.println("Auton"));
  }

}