// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.opencv.video.Video;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.UsbCameraInfo;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.libs.swerveyshark.sharkexe.SharkExecutor;
// import frc.robot.commands.BlueOneConeMobile;
import frc.robot.subsystems.DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static RobotContainer m_robotContainer;

  public enum Phase {
    AUTON,
    TELEOP,
    DISABLED
  }
  public static Phase state;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    UsbCamera camera = CameraServer.startAutomaticCapture(0);
    camera.setResolution(400, 380);
    camera.setExposureManual(50);
    //camera.setFPS(30);
    m_robotContainer = new RobotContainer();
    state = Phase.DISABLED;
    try {
      SharkExecutor.loadAndConfigurePath("blue3nb", "/paths/blue_three_piece_nobump_wip.csv", (target) -> DriveTrain.getInstance().toPose(target));
      // SharkExecutor.loadAndConfigurePath("red3nb", "/paths/red_three_piece_nobump_wip.csv", (target) -> DriveTrain.getInstance().toPose(target));
      SharkExecutor.loadAndConfigurePath("blue3b", "/paths/blue_three_piece_bump.csv", (target) -> DriveTrain.getInstance().toPose(target));
      SharkExecutor.loadAndConfigurePath("red3b", "/paths/red_three_piece_bump.csv", (target) -> DriveTrain.getInstance().toPose(target));
      SharkExecutor.loadAndConfigurePath("balls", "/paths/blue_New_New_New_New_Path.csv", (target) -> DriveTrain.getInstance().toPose(target));
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    
     
    // Robot.m_robotContainer.intake.setDistanceSensor(true);
    // Robot.m_robotContainer.intake.setDistanceSensorAuto(true);

    // PortForwarder.add(1181, "hammerheads-jetson.local", 1181);
    // PortForwarder.add(1182, "hammerheads-jetson.local", 1182);
    // m_robotContainer.vision.shutdown();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
   
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
   
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    state = Phase.DISABLED;
    
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    state = Phase.AUTON;

    m_robotContainer.getAutonomousCommand().schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Robot.m_robotContainer.dt.resetFlip();
    // Robot.m_robotContainer.dt.toggleAuto();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    state = Phase.TELEOP;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    CameraServer.startAutomaticCapture(1);
    
    
    //usbCamera.setResolution(0, 0);
    //Sendable cameraSendable = (Sendable) CameraServer.putVideo("Camera Feed", 5, 5);
    //SmartDashboard.putData("Camera Server", (Sendable)usbCamera);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
