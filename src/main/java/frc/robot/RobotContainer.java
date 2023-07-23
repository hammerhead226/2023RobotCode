

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
import frc.robot.commands.BlueOneConeMobile;
import frc.robot.commands.EmptyAuto;
import frc.robot.commands.Level2;
import frc.robot.commands.Level3;
import frc.robot.commands.LimelightLineUp;
import frc.robot.commands.MobilityEngage;
import frc.robot.commands.OneConeEngage;
import frc.robot.commands.OneConeLowMobilityEngage;
import frc.robot.commands.OneConeMobilityEngage;
import frc.robot.commands.OneCubeLow;
import frc.robot.commands.OneCubeMobilityEngage;
import frc.robot.commands.OnePieceLowMobility;
import frc.robot.commands.ScoreConeOrCube;
// import frc.robot.commands.RedOneConeMobile;
import frc.robot.commands.Scoring;
import frc.robot.commands.Stow;
import frc.robot.commands.Substation;
import frc.robot.commands.ThreePieceAutons;
// import frc.robot.commands.OneConeEngage;
import frc.robot.commands.OneConeMobilityEngage;
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

  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(value, max));
  }

  private final double ADJ_SPEED = 0.75;
  
  public RobotContainer() {
    configureBindings();

    dt.setDefaultCommand(
      new RunCommand(
        () -> dt.control(clamp(Math.pow(driver.getLeftJoyX(), 2), 0, ADJ_SPEED) * (driver.getLeftJoyX() < 0 ? -1 : 1),
                         clamp(Math.pow(driver.getLeftJoyY(), 2), 0, ADJ_SPEED) * (driver.getLeftJoyY() < 0 ? -1 : 1), 
                         clamp(Math.pow(driver.getRightJoyX(), 2), 0, ADJ_SPEED) * (driver.getRightJoyX() < 0 ? -1 : 1)),
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

    // gripper.setDefaultCommand(new RunCommand(gripper::run, gripper));
    intake.setDefaultCommand(new RunCommand(intake::run, intake));
    // linearSlide.setDefaultCommand(new RunCommand(linearSlide::run, linearSlide));
    // elevator.setDefaultCommand(new RunCommand(elevator::run, elevator));

    led.setDefaultCommand(new SetColorMode());

    // three pieces
    // rememebr to fill in the csv files from reformatter
    selecter.addOption("red one cone low mobility engage", new OneConeLowMobilityEngage("Red"));
    selecter.addOption("blue one cone low mobility engage", new OneConeLowMobilityEngage("Blue"));

    selecter.addOption("one piece low mobility", new OnePieceLowMobility());

    selecter.addOption("score cone or cube", new ScoreConeOrCube());
    selecter.addOption("score cube low", new OneCubeLow());
    selecter.addOption("mobility engage", new MobilityEngage());
    selecter.addOption("one cone mobile and engage", new OneConeMobilityEngage());
    selecter.addOption("one cube mobile and engage", new OneCubeMobilityEngage());
    selecter.addOption("red three piece no bump", new ThreePieceAutons("red3nb"));
    selecter.addOption("red three piece bump", new ThreePieceAutons("red3b"));
    selecter.addOption("blue three piece no bump", new ThreePieceAutons("blue3nb"));
    selecter.addOption("blue three peice bump", new ThreePieceAutons("blue3b"));

    selecter.addOption("empty", new EmptyAuto());

    // selecter.setDefaultOption("one and engage", new OneConeEngage());
    // selecter.addOption("blue one cone mobile", new BlueOneConeMobile());
    // selecter.addOption("red one cone mobile", new RedOneConeMobile());
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
    driver.getMENUButton().onTrue(new InstantCommand(dt::resetFlip, dt));

    driver.getSTARTButton().onTrue(new InstantCommand(dt::reset, dt));

    driver.getAButton().onTrue(new InstantCommand(intake::retractIntake, intake));
    driver.getXButton().onTrue(new InstantCommand(intake::extendIntake, intake));
    driver.getYButton().onTrue(new InstantCommand(intake::lowerIntake, intake));
    driver.getRBButton().onTrue(new InstantCommand(dt::toggleSpeed, dt));
    // driver.getBButton().whileTrue(new LimelightLineUp());
    // driver.getBButton().whileTrue(new RunCommand(() -> dt.control(0, 0.2, 0)));
    // driver.getBButton().onFalse(new InstantCommand(() -> dt.control(0, 0, 0)));


    manip.getYButton().onTrue(new Level3());
    manip.getBButton().onTrue(new Level2());
    manip.getAButton().onTrue(new Stow());

    //make is so when slide is fully in after scoring akul controller buzzes
    manip.getRBButton().onTrue(new Scoring());
    manip.getRightStickPress().onTrue(new Substation());
    
    manip.getLBButton().onTrue(new InstantCommand(intake::runIn));
    manip.getLBButton().onFalse(new InstantCommand(intake::stop));

    manip.getMENUButton().onTrue(new InstantCommand(intake::runOut));
    manip.getMENUButton().onFalse(new InstantCommand(intake::stop));


    manip.getSTARTButton().onTrue(new InstantCommand(gripper::toggleCubeMode, gripper));
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