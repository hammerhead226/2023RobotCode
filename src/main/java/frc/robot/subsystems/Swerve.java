// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.electronics.IMU.Gyro;
import frc.libs.electronics.IMU.Pigeon2IMU;
import frc.libs.whirlpool.SwerveModule;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {

  public static final Swerve INSTANCE = new Swerve(null);

  public static Swerve getInstance() {
    return INSTANCE;
  }

  public SwerveDriveOdometry odometry;
  public SwerveDriveKinematics kinematics;
  public SwerveModule[] modules;
  public Gyro<?> gyro;

  public Swerve(Gyro<?> gyro) {
    this.gyro = gyro;
    gyro.reset();

    for (int i = 0; i < Constants.NUMBER_OF_MODULES; i++) {
      modules[i] = new SwerveModule(i);
    }

    kinematics = new SwerveDriveKinematics(new Translation2d(Constants.LENGTH, Constants.WIDTH),
                                           new Translation2d(Constants.LENGTH, -Constants.WIDTH),
                                           new Translation2d(-Constants.LENGTH, Constants.WIDTH),
                                           new Translation2d(-Constants.LENGTH, -Constants.WIDTH));     

    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyro.getYaw()), getModuleStates());
  }

  public Swerve() {
    this.gyro = new Pigeon2IMU(0, Constants.CANBUS);
    gyro.reset();

    for (int i = 0; i < Constants.NUMBER_OF_MODULES; i++) {
      modules[i] = new SwerveModule(i);
    }

    kinematics = new SwerveDriveKinematics(new Translation2d(Constants.LENGTH, Constants.WIDTH),
                                           new Translation2d(Constants.LENGTH, -Constants.WIDTH),
                                           new Translation2d(-Constants.LENGTH, Constants.WIDTH),
                                           new Translation2d(-Constants.LENGTH, -Constants.WIDTH));     

    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyro.getYaw()), getModuleStates());
  }

  public SwerveModulePosition[] getModuleStates(){
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for(SwerveModule mod : modules){
        states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void control(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        kinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                new Rotation2d(gyro.getYaw())
                            )
                            : new ChassisSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation)
                            );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 5);

    for(SwerveModule mod : modules){
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }    

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 5);

    for (SwerveModule mod : modules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
