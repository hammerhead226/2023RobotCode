// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Controller Configuration
  public static final double CONTROLLER_DEADBAND = 0.15;

  public static final String CANBUS = "CAN Bus 2";

  public static final double DRIVETRAIN_TILT_THRESHOLD = 15;

  //Swervey Configuration Parameters
 
     // Encoder Parameters
     public static final double OVERFLOW_THRESHOLD = Math.PI;
 
     public static final double[] MODULE_OFFSETS = {22.5, 313.857421875, 21.26953125, 123.92578125};
 
 
     // Physical Configuration 
     public static final int NUMBER_OF_MODULES = 4;
     public static final double LENGTH = 22.8125;
     public static final double WIDTH = 22.8125;
 
 
     public static final double TRANSLATIONAL_ERROR = 3;
     public static final double ROTATE_ERROR = Math.PI/24;
     public static final double LIMELIGHT_OFFSET = 0;
 
     public static final double TICKS_PER_INCHES = 1170.58602;//44.228775
 
     public static String AUTONOMOUS_PATH_FILENAME = "/paths/recording.csv";
 
     // Teleoperated Parameters
     public static final double LOW_BOUND_SPEED = 0.7;
     public static final double HIGH_BOUND_SPEED = 0.85;
 
     public static final double ACCELERATION_RATE = 0.01;
     public static final double INITIAL_SPEED = 0.5;
 
     // Drivetrain Current Parameters
     public static final boolean DRIVETRAIN_CURRENT_ENABLE = false;
     public static final double DRIVETRAIN_CURRENT_LIMIT = 40;
     public static final double DRIVETRAIN_CURRENT_THRESHOLD_TIME = 2;
     public static final double DRIVETRAIN_CURRENT_THRESHOLD_LIMIT = 60;
 
     // Drivetrain Voltage Parameters
     public static final double DRIVETRAIN_VOLTAGE_LIMIT = 12;
     public static final boolean DRIVETRAIN_VOLTAGE_ENABLE = false;
 
 
     // PID Configurations
     public static final double[] DRIVE_GAINS = {0.04, 0.0, 0.0}; //0.7115, 0.6, .18
 
     public static final double[] STEER_GAINS_LOW = {0.62, 0, 0.0};
     public static final double[] STEER_GAINS_HIGH = {.68, 0, 0.0};
 
     public static final double[] ROTATE_GAINS_LOW = {1.5, 0.0, 0.0};//.669, 2.5
     public static final double[] ROTATE_GAINS_HIGH = {0.4, 0.0, 0.0};//.9
 
     public static double[] LIMELIGHT_GAINS = {0.065, 0.0, 0.0};
 
     public static final double STEER_GAINS_THRESHOLD = 2146.5; // 9%
     public static final double ROTATE_GAINS_THRESHOLD = 0.15; //TODO: Convert to velocity
     public static final double ROTATE_VELOCITY_THRESHOLD = 0.2;

     public static final double VELOCITY_FEED_FORWARD = 0;
 
 
     // REFORMATTED SWERVE PARAMETERS
     public static final double[][] MODULE_GAINS = {DRIVE_GAINS,
                                                    STEER_GAINS_LOW,
                                                    ROTATE_GAINS_LOW};
                                                  
     public static final double[][] SCHEDULED_GAINS = {STEER_GAINS_HIGH,
                                                       ROTATE_GAINS_HIGH};
 
     public static final double[] STEER_AND_ROTATE_THRESHOLDS = {STEER_GAINS_THRESHOLD, ROTATE_GAINS_THRESHOLD, ROTATE_VELOCITY_THRESHOLD};
 
     public static final double[][] MODULE_POSITIONS =  {new double[]{WIDTH/2, LENGTH/2}, 
                                                         new double[]{-WIDTH/2, LENGTH/2}, 
                                                         new double[]{-WIDTH/2, -LENGTH/2}, 
                                                         new double[]{WIDTH/2, -LENGTH/2}};
 
     public static final double[] ALLOWED_ERRORS = {TRANSLATIONAL_ERROR, ROTATE_ERROR};
 
     public static final double[] SPEED_BOUNDS = {HIGH_BOUND_SPEED, LOW_BOUND_SPEED};
 
     public static final double[] ACCELERATION_PARAMETERS = {ACCELERATION_RATE, INITIAL_SPEED};

     public static final double[] LIMELIGHT_COEFS = {0, 0, 0, 0};

  // Elevator Constants
  public static final double[] ELEVATOR_GAINS = { 0.00125, 0, 0 };

  public static final int SLIDE_DISABLE_POSE = 1500;
  public static final int ELEVATOR_MIN = -300;
  public static final int ELEVATOR_MAX = 2000;

  public static final double ELEVATOR_INTERVAL_MARKER = 2800;
  public static final double SRX_ENCODER_TICKS = 4096;

  public static final double ELEVATOR_THRESHOLD = 100;

  public static final double ELEVATOR_STOW = 0;


  // Linear Slide Constants
  public static final double[] LINEAR_SLIDE_GAINS_HIGH = { 0.0000125, 0.00001, 0 };
  public static final double[] LINEAR_SLIDE_GAINS_LOW = { 0.000025, 0.00001, 0 };
  public static final boolean LS_SET_INVERTED = true;
  public static final double LS_THRESHOLD = 2000;


  // LED Constants
  public static final double LED_ALLIANCE = 0.87; //blue
  public static final double LED_ALLIANCE_BLUE = 0.87; //blue
  public static final double LED_ALLIANCE_RED = 0.61;

  public static final double LED_YELLOW = 0.66;
  public static final double LED_VIOLET = 0.91;


  // Intake Constants
  public static final double ROLLER_RUN_SPEED = 0.65;
  public static final double[] INTAKE_GAINS = { 0.0009, 0, 0 };
  public static final int INTAKE_OFFSET = 0;

  public static final double INTAKE_EXTEND = 3560;
  public static final double INTAKE_RETRACT = 2600;

  public static final double INTAKE_THRESHOLD = 100;
      
  // 3000 to be fully retracted
  
  public static final double MAX_SPEED_UP = 0.5; //extend
  public static final double MAX_SPEED_DOWN = 0.8; //retract


  // Gripper Constants
  public static final double ARM_POS_1 = 0;
  public static final double ARM_POS_2 = -130000;

  // public static final double ARM_STOW = 0;
  // public static final double ARM_HOLD = 0;

  public static final double[] ARM_GAINS = {0.000015, 0, 0};
  public static final double[] CLAW_GAINS = {0.00003, 0, 0};

  public static final double ARM_THRESHOLD = 2000;

  public static final double CLOSING_DISTANCE = 8;

  public static final double CUBE_VALUE = 1350;
  public static final double CONE_VALUE = 2100;


  //Scoring Setpoints
  public static final double ELEVATOR_HIGH = -1200;
  public static final double ELEVATOR_MID = -200;
  public static final double ELEVATOR_HOLD = 1900;
  public static final double ELEVATOR_INTAKE = 1100;
  public static final double ELEVATOR_SUBSTATION = -550;

  public static final double LS_HIGH = 50000;
  public static final double LS_MID = 18000;
  public static final double LS_RETRACTED = 0;
  public static final double LS_SUBSTATION = 17;

  public static final double ARM_HOLD = 80900;
  public static final double ARM_SCORE = 104000;

  public static final double ARM_STOW = -850;

  public static final double ARM_SUBSTATION = -107500;
  public static final double ARM_INTAKE = -43000;
  public static final double ARM_CONE_HOLD = 3000;
  public static final double ARM_CUBE_HOLD = 0;


}
