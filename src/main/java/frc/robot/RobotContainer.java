

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LED;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

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
import frc.robot.commands.OneAndHalfPieceMobility;
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
import frc.robot.commands.ballsballs;
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
import frc.robot.commands.Handoff;
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

  private final double ADJ_SPEED = 0.65;
  
  public RobotContainer() {
    configureBindings();

    dt.setDefaultCommand(
      new RunCommand(
        () -> dt.control(clamp(Math.pow(driver.getLeftJoyX(), 2), 0, ADJ_SPEED) * (driver.getLeftJoyX() < 0 ? -1 : 1),
                         clamp(Math.pow(driver.getLeftJoyY(), 2), 0, ADJ_SPEED) * (driver.getLeftJoyY() < 0 ? -1 : 1), 
                         -clamp(Math.pow(driver.getRightJoyX(), 2), 0, ADJ_SPEED) * (driver.getRightJoyX() < 0 ? -1 : 1)),
        dt
        ));

    // linearSlide.setDefaultCommand(
    //   new RunCommand(() -> linearSlide.control(clamp(manip.getLeftJoyY(), -0.6, 0.6)), linearSlide)
    // );
    
    // elevator.setDefaultCommand(
    //   new RunCommand(() -> elevator.control(clamp(manip.getLeftJoyY(), -0.6, 0.6)), elevator)
    // );

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

    // TODO: Check if this type cast is legit
    // PathPlannerTrajectory path = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));

    // // TODO: Implement Event Map
    // // This is just an example event map. It would be better to have a constant, global event map
    // // in your code that will be used by all path following commands.
    // HashMap<String, Command> eventMap = new HashMap<>();
    // // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    // // eventMap.put("intakeDown", new IntakeDown());

    // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    //     dt::getPose, // Pose2d supplier
    //     dt::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
    //     dt.kinematics, // SwerveDriveKinematics
    //     new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    //     new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    //     dt::setModuleStates, // Module states consumer used to output to the drive subsystem
    //     eventMap,
    //     true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    //     dt // The drive subsystem. Used to properly set the requirements of path following commands
    // );

    // Command fullAuto = autoBuilder.fullAuto(path);

    // three pieces
    // rememebr to fill in the csv files from reformatter

    selecter.addOption("one and half piece mobility", new OneAndHalfPieceMobility());
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

    selecter.addOption("balls", new ballsballs("balls"));

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

    // driver.getAButton().onTrue(new InstantCommand(intake::retractIntake, intake));
    // driver.getXButton().onTrue(new InstantCommand(intake::extendIntake, intake));
    // driver.getYButton().onTrue(new InstantCommand(intake::lowerIntake, intake));
    // driver.getXButton().onTrue(new InstantCommand(intake::extendIntake, intake));
    // driver.getXButton().onTrue(new InstantCommand(intake::runIn, intake));
    // driver.getXButton().onFalse(new InstantCommand(intake::retractIntake, intake));
    // driver.getXButton().onFalse(new InstantCommand(intake::stop, intake));

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

    driver.getAButton().onTrue(new InstantCommand(intake::runOut, intake));
    driver.getAButton().onFalse(new InstantCommand(intake::stop, intake));

    driver.getYButton().onTrue(new InstantCommand(intake::runIn, intake));
    driver.getYButton().onFalse(new InstantCommand(intake::deadStop, intake));

    driver.getYButton().onTrue(new InstantCommand(intake::extendIntake, intake));
    driver.getYButton().onFalse(new InstantCommand(intake::retractIntake, intake));

    // manip.getAButton().onTrue(new Handoff());

    // manip.getBButton().onTrue(new InstantCommand(gripper::toggleBrakeMode));
    // manip.getAButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(Constants.LS_MID), linearSlide));
    // manip.getBButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(Constants.LS_RETRACTED), linearSlide));
    // manip.getYButton().onTrue(new InstantCommand(() -> linearSlide.setTarget(Constants.LS_HIGH), linearSlide));

    // manip.getAButton().onTrue(new InstantCommand(() -> elevator.setTarget(Constants.ELEVATOR_HIGH), elevator));
    // manip.getBButton().onTrue(new InstantCommand(() -> elevator.setTarget(Constants.ELEVATOR_MID), elevator));
    // manip.getYButton().onTrue(new InstantCommand(() -> elevator.setTarget(Constants.ELEVATOR_HOLD), elevator));
    // manip.getXButton().onTrue(new InstantCommand(() -> elevator.setTarget(Constants.ELEVATOR_SUBSTATION), elevator));

    // manip.getAButton().onTrue(new InstantCommand(() -> gripper.setArmTarget(Constants.ARM_STOW), gripper));
    // manip.getBButton().onTrue(new InstantCommand(() -> gripper.setArmTarget(Constants.ARM_SCORE), gripper));
    // manip.getYButton().onTrue(new InstantCommand(() -> gripper.setArmTarget(Constants.ARM_HOLD), gripper));
    // manip.getXButton().onTrue(new InstantCommand(() -> gripper.setArmTarget(Constants.ARM_SUBSTATION), gripper));
    // manip.getSTARTButton().onTrue(new InstantCommand(gripper::toggleCubeMode, gripper));
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