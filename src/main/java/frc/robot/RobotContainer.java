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

    manip.getSTARTButton().onTrue(new InstantCommand(gripper::toggleCubeMode, gripper));

    manip.getRBButton().onTrue(new InstantCommand(gripper::toggleClaw, gripper));


    manip.getLBButton().onTrue(
      new InstantCommand(
        intake::extendIntake, intake
      )
      .andThen(intake::runIn, intake)
      .andThen(activeFloor::runConstantSpeedInward, activeFloor)
    );

    manip.getLBButton().onFalse(
      new InstantCommand(
        intake::retractIntake, intake
      )
      .andThen(intake::stop, intake)
      .andThen(activeFloor::stop, activeFloor)
    );

    manip.getMENUButton().onTrue(
      new InstantCommand(
        intake::extendIntake, intake
      )
      .andThen(intake::runOut, intake)
      .andThen(activeFloor::runConstantSpeedOutward, activeFloor)
    );

    manip.getMENUButton().onFalse(
      new InstantCommand(
        intake::retractIntake, intake
      )
      .andThen(intake::stop, intake)
      .andThen(activeFloor::stop, activeFloor)
    );
    manip.getXButton().onTrue(new InstantCommand(gripper::toggleArm, gripper));


    manip.getLeftStickPress().onTrue(new InstantCommand(gripper::toggleWrist, gripper));
    manip.getAButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(0), linearSlide)
    .andThen(() -> elevator.setTarget(1300), elevator));

    manip.getBButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(30), linearSlide)
    .andThen(() -> elevator.setTarget(1000), elevator));
    manip.getYButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(47), linearSlide)
    .andThen(() -> elevator.setTarget(0), elevator));
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