// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.swervey.Swerve;
import frc.libs.swervey.SwerveBuilder;
import frc.libs.wrappers.GenericEncoder;
import frc.libs.wrappers.GenericMotor;
import frc.libs.wrappers.Gyro;
import frc.libs.wrappers.LimeLight;
import frc.libs.wrappers.SharkSight;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class DriveTrain extends SubsystemBase {
  private final static DriveTrain INSTANCE = new DriveTrain();

  public static DriveTrain getInstance() {
    return INSTANCE;
  }

  private Swerve swerve;
  private boolean driveTrainLock = false;
  private PIDController limelightController;
  private PIDController jetsonController;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    GenericMotor[] drives = new GenericMotor[Constants.NUMBER_OF_MODULES];
    GenericMotor[] steers = new GenericMotor[Constants.NUMBER_OF_MODULES];
    GenericEncoder[] encoders = new GenericEncoder[Constants.NUMBER_OF_MODULES];

    for (int i = 0; i < Constants.NUMBER_OF_MODULES; i++) {
      TalonFX drive = new TalonFX(RobotMap.DRIVE_MOTORS[i]);
      TalonFX steer = new TalonFX(RobotMap.STEER_MOTORS[i]);
      CANCoder encoder = new CANCoder(RobotMap.ENCODERS[i]);

      steer.setInverted(true);

      drives[i] = new GenericMotor(drive);
      steers[i] = new GenericMotor(steer);
      encoders[i] = new GenericEncoder(encoder, Constants.OVERFLOW_THRESHOLD, Constants.MODULE_OFFSETS[i]);
    }

    Gyro gyro = new Gyro(RobotMap.GYRO);

    swerve = new SwerveBuilder(drives, steers, encoders, gyro)
        .PIDGains(Constants.MODULE_GAINS, Constants.SCHEDULED_GAINS, Constants.STEER_AND_ROTATE_THRESHOLDS)
        .modulePositions(Constants.MODULE_POSITIONS)
        .speedBounds(Constants.SPEED_BOUNDS)
        .accelerationParameters(Constants.ACCELERATION_PARAMETERS)
        .autonomousParameters(Constants.TICKS_PER_INCHES, Constants.ALLOWED_ERRORS)
        .buildSwerve();

    this.limelightController = new PIDController(Constants.LIMELIGHT_GAINS[0], Constants.LIMELIGHT_GAINS[1],
        Constants.LIMELIGHT_GAINS[2]);
    this.limelightController.setTolerance(0.9);

    this.jetsonController = new PIDController(Constants.SHARKSIGHT_INTAKE_GAINS[0],
        Constants.SHARKSIGHT_INTAKE_GAINS[1], Constants.SHARKSIGHT_INTAKE_GAINS[2]);
    this.jetsonController.setTolerance(0.9);
    // swerve.enableRobotCentric();
  }

  public void control(double x, double y, double rotate) {
    swerve.control(x, y, rotate);
  }

  public void jetsonLineUp(double angle) {
    if (SharkSight.isEnabled() && SharkSight.getClosestIntakeDetect() != null) {
      control(0, 0,
          Math.abs(SharkSight.getClosestIntakeDetect().targetX) >= Constants.SHARKSIGHT_ERROR_MARGIN
              ? jetsonController.calculate(SharkSight.getClosestIntakeDetect().targetX, angle)
              : 0);
    }
  }

  public void limelightLineUp(boolean isMove, double offset) {
    if (!isMove) {
      control(Math.abs(SharkSight.getClosestIntakeDetect().targetX) >= Constants.SHARKSIGHT_ERROR_MARGIN
          ? limelightController.calculate(SharkSight.getClosestIntakeDetect().targetX, offset)
          : 0,
          0, 0);
    }
  }

  public void toggleSpeed() {
    swerve.toggleSpeed();
  }

  public double getCurrentSpeedMultiplier() {
    return swerve.getCurrentSpeedMultiplier();
  }

  public void toPose(double[] pose) {
    swerve.toPose(pose);
  }

  public void setTarget(double[] target) {
    swerve.setTargetPosition(target);
  }

  public boolean atSetpoint() {
    return swerve.atSetpoint();
  }

  public void reset() {
    swerve.zeroGyro();
    swerve.reset();
  }

  public void lockDriveTrain() {
    this.driveTrainLock = true;
  }

  public void unlockDriveTrain() {
    this.driveTrainLock = false;
  }

  public PIDController getRotationPIDController() {
    return limelightController;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (int i = 0; i < Constants.NUMBER_OF_MODULES; i++) {
      SmartDashboard.putNumber("module offset " + i, swerve.getModuleRotationalPose(i));
    }
  }
}
