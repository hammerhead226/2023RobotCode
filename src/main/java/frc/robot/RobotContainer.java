// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LinearSlide;
import edu.wpi.first.wpilibj2.command.RunCommand;


import frc.libs.wrappers.Controller;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ActiveFloor;

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

  private final ActiveFloor activeFloor = new ActiveFloor();


  // The robot's subsystems and commands are defined here...
  private final Intake intake = new Intake();
  private final LinearSlide linearSlide = new LinearSlide();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */


  private final Controller driver = new Controller(0, Constants.CONTROLLER_DEADBAND);

  public LED led = new LED();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    intake.setDefaultCommand(new RunCommand(intake::run, intake));
    linearSlide.setDefaultCommand(new RunCommand(linearSlide::run, linearSlide));
  }

  
  private void configureBindings() {



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