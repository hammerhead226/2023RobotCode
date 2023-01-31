// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
    // Controller Configuration
    public static final double CONTROLLER_DEADBAND = 0.15;

    //Swervey Configuration Parameters

    // Encoder Parameters
    public static final double OVERFLOW_THRESHOLD = Math.PI;

    public static final double[] MODULE_OFFSETS = {30.146484375, 317.98828125, 26.89453125, 122.607421875};


    // Physical Configuration 
    public static final int NUMBER_OF_MODULES = 4;
    public static final double LENGTH = 22.8125;
    public static final double WIDTH = 22.8125;


    // TODO: Autonomous Parameters
    public static final double TRANSLATIONAL_ERROR = 0;
    public static final double ROTATE_ERROR = 0;
    public static final double LIMELIGHT_OFFSET = 0;

    public static final double TICKS_PER_INCHES = 0;

    public static String AUTONOMOUS_PATH_FILENAME = "/paths/recording.csv";
    // public static String AUTONOMOUS_PATH_FILENAME = "/paths/2Balls1HangarFinalv2.csv";


    // Teleoperated Parameters
    public static final double LOW_BOUND_SPEED = 0.75;
    public static final double HIGH_BOUND_SPEED = 0.85;

    public static final double ACCELERATION_RATE = 0.01;
    public static final double INITIAL_SPEED = 0.3;

    // Drivetrain Current Parameters
    public static final boolean DRIVETRAIN_CURRENT_ENABLE = false;
    public static final double DRIVETRAIN_CURRENT_LIMIT = 40;
    public static final double DRIVETRAIN_CURRENT_THRESHOLD_TIME = 2;
    public static final double DRIVETRAIN_CURRENT_THRESHOLD_LIMIT = 60;

    // Drivetrain Voltage Parameters
    public static final double DRIVETRAIN_VOLTAGE_LIMIT = 12;
    public static final boolean DRIVETRAIN_VOLTAGE_ENABLE = false;


    // PID Configurations
    public static final double[] DRIVE_GAINS = {0.7115, 0.0, 0.0};

    public static final double[] STEER_GAINS_LOW = {0.62, 0, 0.0};
    public static final double[] STEER_GAINS_HIGH = {.68, 0, 0.0};

    public static final double[] ROTATE_GAINS_LOW = {0.669, 0.0, 0.0};
    public static final double[] ROTATE_GAINS_HIGH = {0.9, 0.0, 0.0};

    public static double[] LIMELIGHT_GAINS = {0.065, 0.0, 0.0};

    public static final double STEER_GAINS_THRESHOLD = 2146.5; // 9%
    public static final double ROTATE_GAINS_THRESHOLD = 0.15; //TODO: Convert to velocity
    public static final double ROTATE_VELOCITY_THRESHOLD = 0.2;


    // REFORMATTED SWERVE PARAMETERS
    public static final double[][] MODULE_GAINS = {DRIVE_GAINS,
                                                   STEER_GAINS_LOW,
                                                   ROTATE_GAINS_LOW};
                                                 
    public static final double[][] SCHEDULED_GAINS = {STEER_GAINS_HIGH,
                                                      ROTATE_GAINS_HIGH};

    public static final double[] STEER_AND_ROTATE_THRESHOLDS = {STEER_GAINS_THRESHOLD, ROTATE_GAINS_THRESHOLD, ROTATE_VELOCITY_THRESHOLD};

    public static final double[][] MODULE_POSITIONS =  {new double[]{WIDTH/24, LENGTH/24}, 
                                                        new double[]{-WIDTH/24, LENGTH/24}, 
                                                        new double[]{-WIDTH/24, -LENGTH/24}, 
                                                        new double[]{WIDTH/24, -LENGTH/24}};

    public static final double[] ALLOWED_ERRORS = {TRANSLATIONAL_ERROR, ROTATE_ERROR};

    public static final double[] SPEED_BOUNDS = {HIGH_BOUND_SPEED, LOW_BOUND_SPEED};

    public static final double[] ACCELERATION_PARAMETERS = {ACCELERATION_RATE, INITIAL_SPEED};

    public static final double[] LIMELIGHT_COEFS = {0, 0, 0, 0};



    //Intake Constants
    public static final double ROLLER_RUN_SPEED = 0.5; 
    public static final double[] INTAKE_GAINS = {0,0,0};
    public static final int INTAKE_OFFSET =;
     
}
