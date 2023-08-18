// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.electronics.IMU.Gyro;
import frc.libs.electronics.IMU.Pigeon2IMU;
import frc.libs.electronics.encoders.ThreadedCANcoder;
import frc.libs.electronics.motors.LazyTalonFX;
import frc.libs.whirlpool.SwerveModule;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Swerve extends SubsystemBase {

  public static final Swerve INSTANCE = new Swerve();

  private static final boolean angleEnableCurrentLimit = false;
  private static final double angleContinuousCurrentLimit = 0;
  private static final double anglePeakCurrentLimit = 0;
  private static final double anglePeakCurrentDuration = 0;

  private static final boolean CANCODER_INVERT = true;

  public static Swerve getInstance() {
    return INSTANCE;
  }

  public SwerveDriveOdometry odometry;
  public SwerveDriveKinematics kinematics;
  public SwerveModule[] modules = new SwerveModule[4];
  public Gyro<?> gyro;

  // public Swerve(Gyro<?> gyro) {
  // this.gyro = gyro;
  // gyro.reset();

  // for (int i = 0; i < Constants.NUMBER_OF_MODULES; i++) {
  // modules[i] = new SwerveModule(i);
  // }

  // kinematics = new SwerveDriveKinematics(new Translation2d(Constants.LENGTH,
  // Constants.WIDTH),
  // new Translation2d(Constants.LENGTH, -Constants.WIDTH),
  // new Translation2d(-Constants.LENGTH, Constants.WIDTH),
  // new Translation2d(-Constants.LENGTH, -Constants.WIDTH));

  // odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyro.getYaw()),
  // getModuleStates());
  // }

  public Swerve() {
    TalonFX[] drives = new TalonFX[Constants.NUMBER_OF_MODULES];
    TalonFX[] steers = new TalonFX[Constants.NUMBER_OF_MODULES];
    CANCoder[] encoders = new CANCoder[Constants.NUMBER_OF_MODULES];

    for (int i = 0; i < Constants.NUMBER_OF_MODULES; i++) {
      TalonFX drive = new TalonFX(RobotMap.DRIVE_MOTORS[i], Constants.CANBUS);
      TalonFX steer = new TalonFX(RobotMap.STEER_MOTORS[i], Constants.CANBUS);

      // drive.configOpenloopRamp(0.1);
      // drive.configClosedloopRamp(0.1);
      // drive.setNeutralMode(NeutralMode.Brake);

      // ANGLE MOTOR CONFIGURATION
      TalonFXConfiguration angleConfig = new TalonFXConfiguration();
      SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
          angleEnableCurrentLimit,
          angleContinuousCurrentLimit,
          anglePeakCurrentLimit,
          anglePeakCurrentDuration);
      angleConfig.slot0.kP = 0.2;
      angleConfig.slot0.kI = 0.0;
      angleConfig.slot0.kD = 0.0;
      angleConfig.slot0.kF = 0.0;
      angleConfig.supplyCurrLimit = angleSupplyLimit;

      // DRIVE MOTOR CONFIGURATION
      TalonFXConfiguration driveConfig = new TalonFXConfiguration();
      driveConfig.slot0.kP = 0.05; // TODO: Tune this
      driveConfig.slot0.kI = 0; 
      driveConfig.slot0.kD = 0;
      driveConfig.slot0.kF = 0;        
      // driveConfig.supplyCurrLimit = driveSupplyLimit;
      driveConfig.openloopRamp = 0.25;
      driveConfig.closedloopRamp = 0;

      // CANCODER CONFIGURATION
      CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
      cancoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
      cancoderConfig.sensorDirection = CANCODER_INVERT;
      cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

      drive.configFactoryDefault();
      steer.configFactoryDefault();
      drive.configAllSettings(driveConfig);
      steer.configAllSettings(angleConfig);
      drive.setNeutralMode(NeutralMode.Brake);
      steer.setNeutralMode(NeutralMode.Brake);

      
      encoders[i] = new CANCoder(i, Constants.CANBUS);
      encoders[i].configFactoryDefault();
      encoders[i].configAllSettings(cancoderConfig);
      
      // encoders[i] = new ThreadedCANcoder(i, Math.PI, Constants.MODULE_OFFSETS[i], 10, Constants.CANBUS, true);

      drives[i] = drive;
      steers[i] = steer;
      // modules[i] = new SwerveModule(i);
    }

    Rotation2d _fl_offset = Rotation2d.fromDegrees(0);
    Rotation2d _fr_offset = Rotation2d.fromDegrees(0);
    Rotation2d _bl_offset = Rotation2d.fromDegrees(0);
    Rotation2d _br_offset = Rotation2d.fromDegrees(0);

    final double MODULE_MAX_ANGULAR_VELOCITY = 2 * Math.PI;
    SwerveModule _fl = new SwerveModule(0, drives[0], steers[0], encoders[0],
        new Translation2d(Constants.LENGTH, Constants.WIDTH), _fl_offset);
    SwerveModule _fr = new SwerveModule(1, drives[1], steers[1], encoders[1],
        new Translation2d(Constants.LENGTH, -Constants.WIDTH),_fr_offset);
    SwerveModule _bl = new SwerveModule(2, drives[2], steers[2], encoders[2],
        new Translation2d(-Constants.LENGTH, Constants.WIDTH), _bl_offset);
    SwerveModule _br = new SwerveModule(3, drives[3], steers[3], encoders[3],
        new Translation2d(-Constants.LENGTH, -Constants.WIDTH), _br_offset);

    _fl.resetToAbsolute();
    _fr.resetToAbsolute();
    _bl.resetToAbsolute();
    _br.resetToAbsolute();

    this.modules[0] = _fl;
    this.modules[1] = _fr;
    this.modules[2] = _bl;
    this.modules[3] = _br;

    this.gyro = new Pigeon2IMU(RobotMap.GYRO, Constants.CANBUS);

    this.gyro.reset();

    // return new SwerveDrive(gyro, 1, _fl, _fr, _bl, _br);

    kinematics = new SwerveDriveKinematics(
        _fl.getModuleLocation(),
        _fr.getModuleLocation(),
        _bl.getModuleLocation(),
        _br.getModuleLocation()
      );

    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyro.getYaw()), getModulePositions());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : modules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }
  
  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : modules){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }
  
  public void control(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SmartDashboard.putNumber("swerve/rotation", rotation);
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            new Rotation2d(gyro.getYaw()))
            : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 5);

    for (SwerveModule mod : modules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 5);

    for (SwerveModule mod : modules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }

    for (int i = 0; i < modules.length; i++) {
      SmartDashboard.putString("swerve/module_" + i + "_desired_angle", desiredStates[i].angle.toString());
    }
  }



  @Override
  public void periodic() {

    for (int i = 0; i < modules.length; i++) {
      SmartDashboard.putString("swerve/module_" + i + "_state", getModuleStates()[i].toString());
    }

  }
}
