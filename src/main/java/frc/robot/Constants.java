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
  public static final double CONTROLLER_DEADBAND = 0.1;

  public static final String CANBUS = "CAN Bus 2";

  public static final double DRIVETRAIN_TILT_THRESHOLD = 15;

  //Swervey Configuration Parameters
 
     // Encoder Parameters
     public static final double OVERFLOW_THRESHOLD = Math.PI;
 
    //  public static final double[] MODULE_OFFSETS = {22.5, 313.857421875, 21.26953125, 123.92578125};

    public static final double[] MODULE_OFFSETS = {22.5 + 180, 313.857421875 + 900, 21.26953125 + 540, 123.92578125+180};
 
 
     // Physical Configuration 
     public static final int NUMBER_OF_MODULES = 4;
     public static final double LENGTH = 0.5794375;
     public static final double WIDTH = 0.5794375;
 
 
     public static final double TRANSLATIONAL_ERROR = 3;
     public static final double ROTATE_ERROR = Math.PI/24;
     public static final double LIMELIGHT_OFFSET = 0;
 
     public static final double TICKS_PER_INCHES = 1170.58602;//44.228775
     public static final double MAX_MODULE_SPEED = 4.96824;
     public static final double TICKS_PER_METER = 196460/MAX_MODULE_SPEED;

 
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

     public static final double[] TRANSLATIONAL_GAINS = {1, 0 , 0};
     public static final double[] ROTATIONAL_GAINS = {0 ,0 ,0 };
 
    //  public static final double[] STEER_GAINS_LOW = {0.62, 0, 0.0};
    //  public static final double[] STEER_GAINS_HIGH = {.68, 0, 0.0};
    public static final double[] STEER_GAINS_LOW = {0.6, 0, 0.0};
     public static final double[] STEER_GAINS_HIGH = {.66, 0, 0.0};
 
    //  public static final double[] ROTATE_GAINS_LOW = {1.5, 0.0, 0.0};//.669, 2.5
    //  public static final double[] ROTATE_GAINS_HIGH = {0.4, 0.0, 0.0};//.9

    public static final double[] ROTATE_GAINS_LOW = {0.8, 0.0, 0.0};//.669, 2.5
     public static final double[] ROTATE_GAINS_HIGH = {0.4, 0.0, 0.0};//.9
 
    //  public static double[] LIMELIGHT_GAINS = {0.065, 0.0, 0.0};
 
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

  public static final double ELEVATOR_THRESHOLD = 8000;



  // Linear Slide Constants
  // public static final double[] LINEAR_SLIDE_GAINS_HIGH = { 0.0000475, 0.0000015, 0 };
  // public static final double[] LINEAR_SLIDE_GAINS_LOW = { 0.0000325, 0.000001, 0 };

  public static final double[] LINEAR_SLIDE_GAINS_HIGH = { 0.000045, 0.00001, 0 };
  public static final double[] LINEAR_SLIDE_GAINS_LOW = { 0.000012, 0.00000125, 0 };
  // public static final double[] LINEAR_SLIDE_GAINS_HIGH = { 0.000052, 0.0000015, 0 };
  // public static final double[] LINEAR_SLIDE_GAINS_LOW = { 0.000043, 0.00000125, 0 };
  public static final boolean LS_SET_INVERTED = true;
  public static final double LS_THRESHOLD = 5000;


  // LED Constants
  public static final double LED_ALLIANCE = 0.87; //blue
  public static final double LED_ALLIANCE_BLUE = 0.87; //blue
  public static final double LED_ALLIANCE_RED = 0.61;

  public static final double LED_YELLOW = 0.66;
  public static final double LED_VIOLET = 0.91;


  // Intake Constants
  public static final double ROLLER_RUN_SPEED = 0.43;
  // public static final double[] INTAKE_GAINS = { 0.0009, 0, 0 };
  // public static final double[] INTAKE_GAINS = {0.0017, 0, 0 };
  // public static final double[] INTAKE_GAINS_RETRACT = {0.0008, 0.0003 , 0};
  // public static final double[] INTAKE_GAINS_EXTEND = {0.0012, 0.0004, 0 };
  public static final double[] INTAKE_GAINS_RETRACT = {0.0005, 0.00005 , 0};
  public static final double[] INTAKE_GAINS_EXTEND = {0.00046, 0.00025, 0 };
  public static final int INTAKE_OFFSET = 0;

  // public static final double INTAKE_EXTEND = -760;
  // public static final double INTAKE_LOWERED = 300 + 2500;
  // public static final double INTAKE_RETRACT = 500;
  public static final double INTAKE_EXTEND = -1500;
  public static final double INTAKE_RETRACT = -100;
  public static final double INTAKE_OUTTAKE = -700;

  public static final double INTAKE_THRESHOLD = 500;
      
  // 3000 to be fully retracted 
  
  public static final double MAX_SPEED_UP = 1; //extend
  public static final double MAX_SPEED_DOWN = -1; //retract


  // Gripper Constants
  public static final double ARM_POS_1 = 0;
  public static final double ARM_POS_2 = -130000;

  public static final double[] ARM_GAINS = {0.00047, 0, 0};
  public static final double[] CLAW_GAINS = {0.00003, 0, 0};

  public static final double ARM_THRESHOLD = 1000;

  public static final double CLOSING_DISTANCE = 8;

  public static final double CUBE_VALUE = 1500;
  public static final double CONE_VALUE = 2300;


  //Scoring Setpoints

  // // old stuff
  // public static final double ELEVATOR_HIGH = -700;
  // public static final double ELEVATOR_MID = 400;
  // // public static final double ELEVATOR_HOLD = 1900;
  // public static final double ELEVATOR_HOLD = 2300;
  // public static final double ELEVATOR_INTAKE = 1700;
  // public static final double ELEVATOR_SUBSTATION = -50;

  public static final double ELEVATOR_HIGH = -500;
  public static final double ELEVATOR_MID = 800;
  public static final double ELEVATOR_HOLD = 1080;
  public static final double ELEVATOR_SUBSTATION = 375;


  // public static final double ELEVATOR_HIGH = -700 + 100;
  // public static final double ELEVATOR_MID = 400 + 100;
  // public static final double ELEVATOR_HOLD = 1900;
  // public static final double ELEVATOR_HOLD = 2300 + 100;
  
  // public static final double ELEVATOR_INTAKE = 1700 + 100;
  // public static final double ELEVATOR_SUBSTATION = -50 + 100;
  // public static final double ELEVATOR_HANDOFF = 1610;

  public static final double LS_HIGH = -60000;
  public static final double LS_MID = -29966;
  public static final double LS_RETRACTED = -600;
  public static final double LS_SUBSTATION = -8180;


  public static final double ARM_HOLD = 5820;
  // public static final double ARM_SCORE = 4680;
  public static final double ARM_SCORE = 4760;
  public static final double ARM_STOW = 7110;
  // public static final double ARM_SUBSTATION = 5420;
  public static final double ARM_SUBSTATION = 5400;
  // public static final double ARM_HANDOFF = 3900;

  // Limelight Constants
  public static final double LIMELIGHT_THRESH = 0.1;
  public static final double ROTATE_THRESH = Math.toRadians(25);
  public static final double TRANSLATION_CUTOFF_THRESH = 0;
  public static final double ROTATE_CUTOFF_THRESH = 0;
  // public static final double[] LIMELIGHT_GAINS = {0.11, 0, 0};



  @Deprecated
  public static final double ARM_INTAKE = -43000;
  public static final double ARM_CONE_HOLD = 3000;
  public static final double ARM_CUBE_HOLD = 0;


}
