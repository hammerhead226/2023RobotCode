// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.Elevator;
import frc.libs.wrappers.Controller;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.OneConeEngage;
import frc.robot.commands.OneConeMobile;
import frc.robot.commands.OneConeMobilityEngage;
import frc.robot.subsystems.ActiveFloor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.balls;

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

  public static final balls lock = new balls();

  SendableChooser<Command> selecter = new SendableChooser<>();
  
  public RobotContainer() {
    configureBindings();

    dt.setDefaultCommand(
      new RunCommand(
        () -> dt.control(driver.getLeftJoyX(), driver.getLeftJoyY(), driver.getRightJoyX()),
        dt
        ));

    gripper.setDefaultCommand(new RunCommand(gripper::run, gripper));
    intake.setDefaultCommand(new RunCommand(intake::run, intake));
    linearSlide.setDefaultCommand(new RunCommand(linearSlide::run, linearSlide));
    elevator.setDefaultCommand(new RunCommand(elevator::run, elevator));

    

    selecter.setDefaultOption("one and engage", new OneConeEngage());
    selecter.addOption("one cone mobile", new OneConeMobile());

    SmartDashboard.putData("auton", selecter);

  }
  
  private void configureBindings() {

    // driver.getLBButton().onTrue(new InstantCommand(led::leftBumperPressed, led));
    // driver.getLBButton().onFalse(new InstantCommand(led::noBumpersPressed, led));

    // driver.getRBButton().onTrue(new InstantCommand(led::rightBumperPressed, led));
    // driver.getRBButton().onFalse(new InstantCommand(led::noBumpersPressed, led));
    driver.getRBButton().onTrue(new InstantCommand(intake::toggleIntake, intake));

    // driver.getAButton().onTrue(new InstantCommand(intake::toggleIntake, intake));
    // driver.getAButton().whileTrue(new RunCommand(() -> dt.toPose(new double[]{0, 20, Math.PI}), dt).until(dt::atSetpoint)
    // .andThen(new RunCommand(() -> dt.toPose(new double[]{0, 100, 0}), dt).until(dt::atSetpoint)));
    // driver.getAButton().whileTrue(new OneConeMobilityEngage());

    driver.getSTARTButton().onTrue(new InstantCommand(dt::reset, dt));
    driver.getLBButton().onTrue(new InstantCommand(() -> dt.toggleSpeed(), dt));
    manip.getSTARTButton().onTrue(new InstantCommand(gripper::toggleCubeMode, gripper));

    manip.getRBButton().onTrue(new InstantCommand(gripper::toggleClaw, gripper));


    manip.getLBButton().onTrue(
      new InstantCommand(intake::runIn, lock)
      .andThen(activeFloor::runConstantSpeedInward, activeFloor)
    );

    manip.getLBButton().onFalse(
      new InstantCommand(intake::stop, lock)
      .andThen(activeFloor::stop, activeFloor)
    );

    manip.getMENUButton().onTrue(
      new InstantCommand(intake::runOut, lock)
      .andThen(activeFloor::runConstantSpeedOutward, activeFloor)
    );

    manip.getMENUButton().onFalse(
      new InstantCommand(intake::stop, lock)
      .andThen(activeFloor::stop, activeFloor)
    );
    manip.getXButton().onTrue(
      new InstantCommand(() -> linearSlide.setTarget(0), lock)
      .andThen(new WaitCommand(0.25))
      .andThen(() -> elevator.setTarget(0), lock)
      .andThen(new WaitCommand(0.25))
      .andThen(gripper::retractArm, lock)
    );


    manip.getLeftStickPress().onTrue(new InstantCommand(gripper::toggleWrist, gripper));

    manip.getAButton().onTrue(
      new InstantCommand(() -> linearSlide.setTarget(0), lock)
      .andThen(() -> gripper.setArmTarget(-10000), lock)
      .andThen(new WaitCommand(0.25))
      .andThen(() -> elevator.setTarget(1125), lock)
      );

      manip.getBButton().onTrue(
        new InstantCommand(() -> elevator.setTarget(0), lock)
        .andThen(new WaitCommand(0.2))
        .andThen(gripper::extendArm, lock)
        .andThen(new WaitCommand(0.75))
        .andThen(() -> linearSlide.setTarget(23), lock));
      
    manip.getYButton().onTrue(
      new InstantCommand(() -> elevator.setTarget(-1100), lock)
      .andThen(new WaitCommand(0.2))
      .andThen(() -> gripper.setArmTarget(-137000), lock)
      .andThen(new WaitCommand(0.75))
      .andThen(() -> linearSlide.setTarget(44), lock));
    
      manip.getRightStickPress().onTrue(
        new InstantCommand(() -> elevator.setTarget(-450), lock)
        .andThen(new WaitCommand(0.25))
        .andThen(() -> gripper.setArmTarget(-115000), lock)
        .andThen(gripper::wristFalconUp, lock)
        .andThen(new WaitCommand(0.5))
        .andThen(() -> linearSlide.setTarget(15), lock)
        );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return (Command) selecter.getSelected();
  }

}