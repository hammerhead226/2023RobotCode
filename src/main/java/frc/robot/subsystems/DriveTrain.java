package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.swervey.MotionOfTheOcean;
import frc.libs.swervey.Swerve;
import frc.libs.swervey.SwerveBuilder;
import frc.libs.wrappers.GenericEncoder;
import frc.libs.wrappers.GenericMotor;
import frc.libs.wrappers.Gyro;
import frc.robot.Constants;
import frc.robot.RobotMap;



public class DriveTrain extends SubsystemBase {

    private final static DriveTrain INSTANCE = new DriveTrain();

    private final PIDController balanceController;

    private final PIDController limelightController;

    private final PIDController rotateController;

    public static DriveTrain getInstance() {
        return INSTANCE;
    }

    private Swerve swerve;

    private Gyro gyro;

    private boolean isPlaying;

    private boolean driveTrainLock;


    private DriveTrain() {

        GenericMotor[] drives = new GenericMotor[Constants.NUMBER_OF_MODULES];
        GenericMotor[] steers = new GenericMotor[Constants.NUMBER_OF_MODULES];
        GenericEncoder[] encoders = new GenericEncoder[Constants.NUMBER_OF_MODULES];

        for(int i = 0; i < Constants.NUMBER_OF_MODULES; i++) {
            TalonFX drive = new TalonFX(RobotMap.DRIVE_MOTORS[i]);
            TalonFX steer = new TalonFX(RobotMap.STEER_MOTORS[i]);

            CANCoder encoder = new CANCoder(RobotMap.ENCODERS[i], Constants.CANBUS);

            drive.setNeutralMode(NeutralMode.Brake);

            drives[i] = new GenericMotor(drive);
            steers[i] = new GenericMotor(steer);

            encoders[i] = new GenericEncoder(encoder, Constants.OVERFLOW_THRESHOLD, Constants.MODULE_OFFSETS[i]);
        }

        gyro = new Gyro(RobotMap.GYRO, "CAN Bus 2");

        swerve = new SwerveBuilder(drives, steers, encoders, gyro)
                .PIDGains(Constants.MODULE_GAINS, Constants.SCHEDULED_GAINS, Constants.STEER_AND_ROTATE_THRESHOLDS)
                .modulePositions(Constants.MODULE_POSITIONS)
                .speedBounds(Constants.SPEED_BOUNDS)
                .accelerationParameters(Constants.ACCELERATION_PARAMETERS)
                .autonomousParameters(Constants.TICKS_PER_INCHES, Constants.ALLOWED_ERRORS, Constants.VELOCITY_FEED_FORWARD)
                .buildSwerve();

        this.balanceController = new PIDController(0.012, 0, 0);

        this.limelightController = new PIDController(0.1, 0, 0);

        // TODO:: play around with this
        this.rotateController = new PIDController(0.4, 0, 0);

        isPlaying = false;


    }

    public void control(double x, double y, double rotate) {
        if(isPlaying) {
            MotionOfTheOcean.Executor.executeRecording(() -> DriveTrain.getInstance().toPose(MotionOfTheOcean.Executor.getState().getPose()));
        }
        else if(!driveTrainLock) swerve.control(x, y, -rotate * 0.75);
        double[] pose = swerve.getPose();
        SmartDashboard.putNumber("x", pose[0]);
        SmartDashboard.putNumber("y", pose[1]);
        SmartDashboard.putNumber("rotate", pose[2]);

        SmartDashboard.putBoolean("atSetpoint", swerve.atSetpoint());
    }

    public void reset() {
        swerve.reset();
        swerve.zeroGyro();
    }

    public void toPose(double[] target) {
        swerve.toPose(target);
    }

    public boolean atSetpoint() {
        return swerve.atSetpoint();
    }

    public void toggleSpeed() {
        swerve.toggleSpeed();
    }

    public void togglePlayback() {
        isPlaying = !isPlaying;
    }

    public void stopPlayback() {
        isPlaying = false;
    }

    public PIDController getBalanceController() {
        return balanceController;
    }

    public PIDController getLimelightController() {
        return this.limelightController;
    }

    public PIDController getRotateController() {
        return this.rotateController;
    }

    public boolean isChassisUnstable() {
        // return Math.abs(gyro.getTilt()) > Constants.DRIVETRAIN_TILT_THRESHOLD;
        return gyro.getTilt() > Constants.DRIVETRAIN_TILT_THRESHOLD || gyro.getTilt() < -9;
    }

    public boolean isChassisStable() {
        return Math.abs(gyro.getTilt()) < 4.5;
    }

    public double getGyroTilt() {
        return gyro.getTilt();
    }

    public double getGyroYaw() {
        return gyro.getYaw();
    }

    public void lockDriveTrain() {
        this.driveTrainLock = true;
    }

    public void unlockDriveTrain() {
        this.driveTrainLock = false;
    }
    
    @Override
    public void periodic() {
      for(int i=0; i < Constants.NUMBER_OF_MODULES; i++)
          SmartDashboard.putNumber("module offset " + i, swerve.getModuleRotationalPose(i));
      SmartDashboard.putNumber("gyro tilt", getGyroTilt());
    }
}

