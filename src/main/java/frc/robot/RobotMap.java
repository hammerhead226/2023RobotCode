package frc.robot;

public class RobotMap {

    // Swerve Drive Ports
    public static final int[] DRIVE_MOTORS = { 0, 1, 2, 3 };
    public static final int[] STEER_MOTORS = { 4, 5, 6, 7 };

    public static final int[] ENCODERS = { 0, 1, 2, 3 };

    public static final int GYRO = 0;


    // Gripper and Arm Ports
    public static final int GRIPPER_WRIST = 18;

    public static final int ARM_MOTOR = 14;
    public static final int WHEELED_CLAW_MOTOR = 15;

    // Active Floor Ports
    public static final int ACTIVE_FLOOR_MOTOR_PORT = 12;

    // Intake Ports
    public static final int INTAKE_PORT = 10;
    public static final int INTAKE_ENCODER_PORT = 16;
    public static final int ROLLER_PORT = 11;

    // Linear Slider Ports 
    public static final int SLIDER_PORT = 13;
    public static final int LED_SPARK = 1;
    public static final int ELEVATOR_MOTOR_LEFT = 8;
    public static final int ELEVATOR_MOTOR_RIGHT = 9;
    public static final int ELEVATOR_ENCODER = 17; 
}
