// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.Elevator;
import frc.libs.wrappers.Controller;
import frc.robot.subsystems.ActiveFloor;
import frc.robot.subsystems.DriveTrain;
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
  public static final Controller driver = new Controller(0, Constants.CONTROLLER_DEADBAND);
  public static final Controller manip = new Controller(1, Constants.CONTROLLER_DEADBAND);
  public static final Controller test = new Controller(2, Constants.CONTROLLER_DEADBAND);

  public static final DriveTrain dt = DriveTrain.getInstance();
  
  public static final Elevator elevator = new Elevator();
  public static final Gripper gripper = new Gripper();
  public static final ActiveFloor activeFloor = new ActiveFloor();
  public static final Intake intake = new Intake();
  public static final LinearSlide linearSlide = new LinearSlide();
  public static final LED led = new LED();
  public static final Vision vision = new Vision();
  
  public RobotContainer() {
    configureBindings();

    dt.setDefaultCommand(
      new RunCommand(
        () -> dt.control(driver.getLeftJoyX(), driver.getLeftJoyY(), driver.getRightJoyX()),
        dt
        ));

    gripper.setDefaultCommand(new RunCommand(gripper::run, gripper));
    // intake.setDefaultCommand(new RunCommand(intake::run, intake));
    linearSlide.setDefaultCommand(new RunCommand(linearSlide::run, linearSlide));
    elevator.setDefaultCommand(new RunCommand(elevator::run, elevator));
  }
  
  private void configureBindings() {

    driver.getLBButton().onTrue(new InstantCommand(led::leftBumperPressed, led));
    driver.getLBButton().onFalse(new InstantCommand(led::noBumpersPressed, led));

    driver.getRBButton().onTrue(new InstantCommand(led::rightBumperPressed, led));
    driver.getRBButton().onFalse(new InstantCommand(led::noBumpersPressed, led));

    driver.getMENUButton().onTrue(new InstantCommand(dt::reset, dt));


    // manip.getLBButton().onTrue(new InstantCommand(intake::toggleIntake, intake));
    // manip.getRBButton().whileTrue(new InstantCommand(intake::runIn, intake).andThen(activeFloor::runConstantSpeedInward, activeFloor));
    // manip.getMENUButton().whileTrue(new InstantCommand(intake::runOut, intake).andThen(activeFloor::runConstantSpeedOutward, activeFloor));


    // manip.getAButton().onTrue(new InstantCommand(gripper::toggleClawCone, gripper));
    // manip.getBButton().onTrue(new InstantCommand(gripper::toggleClawCube, gripper));
    // manip.getXButton().onTrue(new InstantCommand(gripper::toggleWrist, gripper));
    // manip.getYButton().onTrue(new InstantCommand(gripper::toggleArm, gripper));


    // manip.getAButton().onTrue(new InstantCommand(activeFloor::runConstantSpeedInward, activeFloor));
    // manip.getAButton().onFalse(new InstantCommand(activeFloor::stop, activeFloor));
    // manip.getBButton().onTrue(new InstantCommand(() -> elevator.setTarget(1000)));

   
    // manip.getXButton().onTrue(new InstantCommand(activeFloor::runConstantSpeedInward, activeFloor));
    // manip.getXButton().onFalse(new InstantCommand(activeFloor::stop, activeFloor));
    manip.getXButton().onTrue(new InstantCommand(gripper::toggleClawCube, gripper));
    manip.getRBButton().onTrue(new InstantCommand(gripper::toggleWrist, gripper));
    manip.getAButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(0), linearSlide)
    .andThen(() -> elevator.setTarget(1250), elevator)
    .andThen(gripper::retractArm, gripper));
    manip.getBButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(25), linearSlide)
    .andThen(() -> elevator.setTarget(1000), elevator)
    .andThen(gripper::extendArm, gripper));
    manip.getYButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(45), linearSlide)
    .andThen(() -> elevator.setTarget(0), elevator)
    .andThen(gripper::extendArm, gripper));

    // manip.getYButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(0), linearSlide));
    // manip.getXButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(15), linearSlide));
    // manip.getAButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(45), linearSlide));

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