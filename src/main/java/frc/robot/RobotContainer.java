

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.ScoringStateManager;
import frc.robot.subsystems.Elevator;
import frc.libs.wrappers.Controller;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Level2;
import frc.robot.commands.Level3;
import frc.robot.commands.LimelightLineUp;
import frc.robot.commands.Scoring;
import frc.robot.commands.Stow;
import frc.robot.commands.Substation;
// import frc.robot.commands.OneConeEngage;
// import frc.robot.commands.OneConeMobilityEngage;
// import frc.robot.commands.OneCubeMobilityEngage;
// import frc.robot.commands.RedOneConeMobile;
// import frc.robot.commands.RedThreePieceBump;
// import frc.robot.commands.RedThreePieceNoBump;
// import frc.robot.commands.RedTwoPieceBump;
// import frc.robot.commands.RedTwoPieceNoBump;
import frc.robot.commands.SetColorMode;
// import frc.robot.commands.TestAuto;
// import frc.robot.commands.BlueOneConeMobile;
// import frc.robot.commands.BlueThreePieceBump;
// import frc.robot.commands.BlueThreePieceNoBump;
// import frc.robot.commands.BlueTwoPieceBump;
// import frc.robot.commands.BlueTwoPieceNoBump;
import frc.robot.commands.FlashGreen;
// import frc.robot.commands.OneConeEngage;
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
  public static final Intake intake = new Intake();
  public static final LinearSlide linearSlide = new LinearSlide();
  public static final LED led = new LED();
  // public static final Vision vision = new Vision();

  public static final ScoringStateManager manager = new ScoringStateManager();

  public static final balls lock = new balls();
  // public static final ballsTwo lockTwo = new ballsTwo();

  public static final Command animation = new FlashGreen(0.1, 10, led)
                                          .alongWith(new InstantCommand(() -> RobotContainer.driver.getJoystick().setRumble(RumbleType.kBothRumble, 1)))
                                          .andThen(new WaitCommand(2))
                                          .andThen(new InstantCommand(() -> RobotContainer.driver.getJoystick().setRumble(RumbleType.kBothRumble, 0)));

  SendableChooser<Command> selecter = new SendableChooser<>();
  
  public RobotContainer() {
    configureBindings();

    dt.setDefaultCommand(
      new RunCommand(
        () -> dt.control(driver.getLeftJoyX(), driver.getLeftJoyY(), driver.getRightJoyX()),
        dt
        ));

    // linearSlide.setDefaultCommand(
    //   new RunCommand(
    //     () -> linearSlide.control(manip.getRightJoyY()),
    //      linearSlide
    //     ));

    // elevator.setDefaultCommand(
    //   new RunCommand(
    //     () -> elevator.control(manip.getRightJoyY()),
    //      elevator
    //     ));

    gripper.setDefaultCommand(new RunCommand(gripper::run, gripper));
    intake.setDefaultCommand(new RunCommand(intake::run, intake));
    linearSlide.setDefaultCommand(new RunCommand(linearSlide::run, linearSlide));
    elevator.setDefaultCommand(new RunCommand(elevator::run, elevator));

    led.setDefaultCommand(new SetColorMode());

    

    // selecter.setDefaultOption("one and engage", new OneConeEngage());
    // selecter.addOption("blue one cone mobile", new BlueOneConeMobile());
    // selecter.addOption("red one cone mobile", new RedOneConeMobile());
    // selecter.addOption("one cone mobile and engage", new OneConeMobilityEngage());
    // selecter.addOption("one cube mobility engage", new OneCubeMobilityEngage());
    // selecter.addOption("red two piece no bump", new RedTwoPieceNoBump());
    // selecter.addOption("blue two piece no bump", new BlueTwoPieceNoBump());
    // selecter.addOption("red two piece bump", new RedTwoPieceBump());
    // selecter.addOption("blue two piece bump", new BlueTwoPieceBump());
    // selecter.addOption("blue three piece no bump", new BlueThreePieceNoBump());
    // selecter.addOption("red three piece no bump", new RedThreePieceNoBump());
    // selecter.addOption("blue three piece bump", new BlueThreePieceBump());
    // selecter.addOption("red three piece bump", new RedThreePieceBump());
    // selecter.addOption("led test", new FlashGreen());
    // selecter.addOption("test auto", new TestAuto());

    SmartDashboard.putData("auton", selecter);

  }
  
  private void configureBindings() {

    // driver.getLBButton().onTrue(new InstantCommand(led::leftBumperPressed, led));
    // driver.getLBButton().onFalse(new InstantCommand(led::noBumpersPressed, led));

    // driver.getRBButton().onTrue(new InstantCommand(led::rightBumperPressed, led));
    // driver.getRBButton().onFalse(new InstantCommand(led::noBumpersPressed, led));




    // driver.getRBButton().onTrue(new InstantCommand(intake::toggleIntake, intake));







    // driver.getAButton().onTrue(new InstantCommand(intake::toggleIntake, intake));
    // driver.getAButton().whileTrue(new RunCommand(() -> dt.toPose(new double[]{0, 20, Math.PI}), dt).until(dt::atSetpoint)
    // .andThen(new RunCommand(() -> dt.toPose(new double[]{0, 100, 0}), dt).until(dt::atSetpoint)));
    // driver.getAButton().whileTrue(new TestAuto().handleInterrupt(DriveTrain.getInstance()::togglePlayback));

    driver.getSTARTButton().onTrue(new InstantCommand(dt::reset, dt));
    // driver.getLBButton().onTrue(new InstantCommand(() -> dt.toggleSpeed(), dt));

    // driver.getAButton().onTrue(new InstantCommand(intake::retractIntake, intake));

    manip.getYButton().onTrue(new Level3());
    manip.getBButton().onTrue(new Level2());
    manip.getAButton().onTrue(new Stow());

    //make is so when slide is fully in after scoring akul controller buzzes
    manip.getLBButton().onTrue(new Scoring());
    manip.getRightStickPress().onTrue(new Substation());

    // manip.getRBButton().onTrue(new Ins)


    // manip.getAButton().onTrue(new InstantCommand(() -> gripper.setArmTarget(Constants.ARM_HOLD), lock));
    // manip.getBButton().onTrue(new InstantCommand(() -> gripper.setArmTarget(Constants.ARM_STOW), lock));
    // manip.getYButton().onTrue(new InstantCommand(() -> gripper.setArmTarget(Constants.ARM_SCORE), lock));
    // manip.getXButton().onTrue(new InstantCommand(() -> gripper.setArmTarget(Constants.ARM_SUBSTATION), lock));

    // manip.getAButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(Constants.LS_RETRACTED)));
    // manip.getBButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(Constants.LS_HIGH)));
    // manip.getXButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(Constants.LS_MID)));
    // manip.getYButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(Constants.LS_SUBSTATION)));

    // make it so when alignment is done akul controller get buzzed 
    // make it so when akul lets go of alignment button anish contorller gets buzzed
    driver.getBButton().whileTrue(new LimelightLineUp());
    driver.getBButton().onFalse(new InstantCommand(() -> DriveTrain.getInstance().control(0, 0, 0), DriveTrain.getInstance()));
    driver.getAButton().onTrue(new InstantCommand(intake::retractIntake, intake));
    // manip.getAButton().onTrue(new InstantCommand(() -> elevator.setTarget(Constants.ELEVATOR_HIGH), elevator));
    // manip.getBButton().onTrue(new InstantCommand(() -> elevator.setTarget(Constants.ELEVATOR_MID), elevator));
    // manip.getXButton().onTrue(new InstantCommand(() -> elevator.setTarget(1600), elevator));
    // manip.getAButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(Constants.LS_HIGH), linearSlide));
    // manip.getBButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(Constants.LS_MID), linearSlide));
    // manip.getXButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(800), linearSlide));

    // manip.getSTARTButton().onTrue(new InstantCommand(gripper::toggleCubeMode, gripper));

    // manip.getAButton().onTrue(new InstantCommand(gripper::wheeledClawIntake, gripper));
    // manip.getAButton().onFalse(new InstantCommand(gripper::wheeledClawStop, gripper));

    // manip.getBButton().onTrue(new InstantCommand(gripper::wheeledClawOuttake, gripper));
    // manip.getBButton().onFalse(new InstantCommand(gripper::wheeledClawStop, gripper));
    // manip.getRBButton().onTrue(new InstantCommand(gripper::toggleClaw, gripper));


    // manip.getLBButton().onTrue(
    //   new InstantCommand(intake::runIn, intake)
    // );

    // manip.getLBButton().onFalse(
    //   new InstantCommand(intake::stop, intake)
    // );

    // manip.getMENUButton().onTrue(
    //   new InstantCommand(intake::runOut, intake)
    // );

    // manip.getMENUButton().onFalse(
    //   new InstantCommand(intake::stop, intake)
    // );
    // manip.getLeftStickPress().onTrue(
    //   new InstantCommand(() -> linearSlide.setTarget(0), lockTwo)
    //   .andThen(new WaitCommand(0.5))
    //   .andThen(gripper::armHoldPosition, lockTwo)
    //   .andThen(new WaitCommand(0.25))
    //   .andThen(() -> elevator.setTarget(0), lockTwo)
    // );

    // manip.getXButton().onTrue(
    //   new InstantCommand(() -> linearSlide.setTarget(Constants.LS_RETRACTED), lockTwo)
    //   .andThen(gripper::cubeModeOn, lockTwo)
    //   .andThen(new WaitCommand(0.5))
    //   .andThen(gripper::armHoldPosition, lockTwo)
    //   .andThen(new WaitCommand(0.25))
    //   // .andThen(gripper::openClaw, lockTwo) // i think we can keep the wait command for the outtake claw part
    //   .andThen(new WaitCommand(0.5))
    //   .andThen(() -> elevator.setTarget(Constants.ELEVATOR_INTAKE), lockTwo)
    //   .andThen(() -> gripper.setArmTarget(Constants.ARM_INTAKE), lockTwo)
    //   );


    // manip.getLeftStickPress().onTrue(new InstantCommand(gripper::toggleWrist, gripper));

    // manip.getAButton().onTrue(
    //   new InstantCommand(() -> linearSlide.setTarget(Constants.LS_RETRACTED), lockTwo)
    //   .andThen(new WaitCommand(0.65))
    //   .andThen(gripper::armHoldPosition, lockTwo)
    //   .andThen(new WaitCommand(0.25))
    //   .andThen(() -> elevator.setTarget(Constants.ELEVATOR_HOLD), lockTwo)
    //   );

    //   manip.getBButton().onTrue(
    //     new InstantCommand(() -> elevator.setTarget(Constants.ELEVATOR_MID), lockTwo)
    //     .andThen(new WaitCommand(0.1))
    //     .andThen(() -> gripper.armHoldPosition(), lockTwo)
    //     .andThen(new WaitCommand(0.25))
    //     .andThen(() -> gripper.setArmTarget(Constants.ARM_SCORE), lockTwo)
    //     .andThen(new WaitCommand(0.75))
    //     .andThen(() -> linearSlide.setTarget(Constants.LS_MID), lockTwo));
      
    //     manip.getYButton().onTrue(
    //       new InstantCommand(() -> elevator.setTarget(Constants.ELEVATOR_HIGH), lockTwo)
    //       .andThen(new WaitCommand(0.1))
    //       .andThen(() -> gripper.armHoldPosition(), lockTwo)
    //       .andThen(new WaitCommand(0.25))
    //       .andThen(() -> gripper.setArmTarget(Constants.ARM_SCORE), lockTwo)
    //       .andThen(new WaitCommand(0.75))
    //       .andThen(() -> linearSlide.setTarget(Constants.LS_HIGH), lockTwo));
    
    //   manip.getRightStickPress().onTrue(
    //     new InstantCommand(() -> elevator.setTarget(Constants.ELEVATOR_SUBSTATION), lockTwo)
    //       .andThen(new WaitCommand(0.1))
    //       .andThen(() -> gripper.armHoldPosition(), lockTwo)
    //       .andThen(new WaitCommand(0.25))
    //       .andThen(() -> gripper.setDoubleSubstation(), lockTwo)
    //       .andThen(new WaitCommand(0.75))
    //       .andThen(() -> linearSlide.setTarget(Constants.LS_SUBSTATION), lockTwo));
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

  public static LinearSlide getLinearSlide() {
    return linearSlide;
  }

  public static Elevator getElevator() {
    return elevator;
  }

  public static Gripper getGripper() {
    return gripper;
  }

  public static Intake getIntake() {
    return intake;
  }

  public static Controller getManip() {
    return manip;
  }

  public static Controller getDriver() {
    return driver;
  }


}