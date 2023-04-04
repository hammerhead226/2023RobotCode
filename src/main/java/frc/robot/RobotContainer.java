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
import frc.robot.commands.OneConeMobilityEngage;
import frc.robot.commands.RedOneConeMobile;
import frc.robot.commands.TestAuto;
import frc.robot.commands.BlueOneConeMobile;
import frc.robot.commands.OneConeEngage;
import frc.robot.subsystems.ActiveFloor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.balls;
import frc.robot.subsystems.ballsTwo;

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
  public static final ballsTwo lockTwo = new ballsTwo();

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
    selecter.addOption("blue one cone mobile", new BlueOneConeMobile());
    selecter.addOption("red one cone mobile", new RedOneConeMobile());
    selecter.addOption("one cone mobile and engage", new OneConeMobilityEngage());

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
    driver.getAButton().whileTrue(new TestAuto().handleInterrupt(DriveTrain.getInstance()::togglePlayback));

    driver.getSTARTButton().onTrue(new InstantCommand(dt::reset, dt));
    driver.getLBButton().onTrue(new InstantCommand(() -> dt.toggleSpeed(), dt));
    manip.getSTARTButton().onTrue(new InstantCommand(gripper::toggleCubeMode, gripper));


    manip.getRBButton().onTrue(new InstantCommand(gripper::toggleClaw, gripper));


    manip.getLBButton().onTrue(
      new InstantCommand(intake::runIn, intake)
      .andThen(activeFloor::runConstantSpeedInward, activeFloor)
    );

    manip.getLBButton().onFalse(
      new InstantCommand(intake::stop, intake)
      .andThen(activeFloor::stop, activeFloor)
    );

    manip.getMENUButton().onTrue(
      new InstantCommand(intake::runOut, intake)
      .andThen(activeFloor::runConstantSpeedOutward, activeFloor)
    );

    manip.getMENUButton().onFalse(
      new InstantCommand(intake::stop, intake)
      .andThen(activeFloor::stop, activeFloor)
    );
    // manip.getXButton().onTrue(
    //   new InstantCommand(() -> linearSlide.setTarget(0), lockTwo)
    //   .andThen(new WaitCommand(0.5))
    //   .andThen(gripper::armHoldPosition, lockTwo)
    //   .andThen(new WaitCommand(0.25))
    //   .andThen(() -> elevator.setTarget(0), lockTwo)
    // );

    manip.getXButton().onTrue(
      new InstantCommand(() -> linearSlide.setTarget(Constants.LS_RETRACTED), lockTwo)
      .andThen(gripper::setCubeMode, lockTwo)
      .andThen(new WaitCommand(0.5))
      .andThen(gripper::armHoldPosition, lockTwo)
      .andThen(new WaitCommand(0.25))
      .andThen(() -> elevator.setTarget(Constants.ELEVATOR_INTAKE - 200), lockTwo)
      .andThen(gripper::openClaw, lockTwo)
      .andThen(new WaitCommand(0.5))
      .andThen(() -> elevator.setTarget(Constants.ELEVATOR_INTAKE), lockTwo)
      .andThen(() -> gripper.setArmTarget(Constants.ARM_INTAKE), lockTwo)
      );


    manip.getLeftStickPress().onTrue(new InstantCommand(gripper::toggleWrist, gripper));

    manip.getAButton().onTrue(
      new InstantCommand(() -> linearSlide.setTarget(Constants.LS_RETRACTED), lockTwo)
      .andThen(new WaitCommand(0.5))
      .andThen(() -> elevator.setTarget(Constants.ELEVATOR_HOLD - 100), lockTwo)
      .andThen(gripper::armHoldPosition, lockTwo)
      .andThen(new WaitCommand(0.25))
      .andThen(() -> elevator.setTarget(Constants.ELEVATOR_HOLD), lockTwo)
      );

      manip.getBButton().onTrue(
        new InstantCommand(() -> elevator.setTarget(Constants.ELEVATOR_MID), lockTwo)
        .andThen(new WaitCommand(0.1))
        .andThen(() -> gripper.armHoldPosition(), lockTwo)
        .andThen(new WaitCommand(0.25))
        .andThen(() -> gripper.setArmTarget(Constants.ARM_SCORE), lockTwo)
        .andThen(new WaitCommand(0.75))
        .andThen(() -> linearSlide.setTarget(Constants.LS_MID), lockTwo));
      
        manip.getYButton().onTrue(
          new InstantCommand(() -> elevator.setTarget(Constants.ELEVATOR_HIGH), lockTwo)
          .andThen(new WaitCommand(0.1))
          .andThen(() -> gripper.armHoldPosition(), lockTwo)
          .andThen(new WaitCommand(0.25))
          .andThen(() -> gripper.setArmTarget(Constants.ARM_SCORE), lockTwo)
          .andThen(new WaitCommand(0.75))
          .andThen(() -> linearSlide.setTarget(Constants.LS_HIGH), lockTwo));
    
      manip.getRightStickPress().onTrue(
        new InstantCommand(() -> elevator.setTarget(Constants.ELEVATOR_SUBSTATION), lockTwo)
          .andThen(new WaitCommand(0.1))
          .andThen(() -> gripper.armHoldPosition(), lockTwo)
          .andThen(new WaitCommand(0.25))
          .andThen(() -> gripper.setDoubleSubstation(), lockTwo)
          .andThen(new WaitCommand(0.75))
          .andThen(() -> linearSlide.setTarget(Constants.LS_SUBSTATION), lockTwo));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command 
  getAutonomousCommand() {
    // An example command will be run in autonomous
    return (Command) selecter.getSelected();
  }

}