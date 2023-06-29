package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.electronics.IMU.Pigeon;
import frc.libs.electronics.IMU.Pigeon2IMU;
import frc.libs.electronics.encoders.ThreadedCANcoder;
import frc.libs.electronics.motors.LazyTalonFX;
import frc.libs.swerveyshark.MotionOfTheOcean;
import frc.libs.swerveyshark.Swerve;
import frc.libs.swerveyshark.SwerveConfiguration;
import frc.libs.swerveyshark.motionoftheocean.SharkExecutor;
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

    private Pigeon2IMU gyro;

    private boolean isPlaying;

    private boolean driveTrainLock;

    private LazyTalonFX saved;


    private DriveTrain() {

        LazyTalonFX[] drives = new LazyTalonFX[Constants.NUMBER_OF_MODULES];
        LazyTalonFX[] steers = new LazyTalonFX[Constants.NUMBER_OF_MODULES];
        ThreadedCANcoder[] encoders = new ThreadedCANcoder[Constants.NUMBER_OF_MODULES];

        for(int i = 0; i < Constants.NUMBER_OF_MODULES; i++) {
            TalonFX drive = new TalonFX(RobotMap.DRIVE_MOTORS[i]);
            TalonFX steer = new TalonFX(RobotMap.STEER_MOTORS[i]);

            drive.configOpenloopRamp(0.25);

            drives[i] = new LazyTalonFX(drive, Constants.TICKS_PER_METER);
            steers[i] = new LazyTalonFX(steer, Constants.TICKS_PER_METER);

            encoders[i] = new ThreadedCANcoder(i, Math.PI, Constants.MODULE_OFFSETS[i], 10, Constants.CANBUS, false);
        }

        saved=drives[0];

        gyro = new Pigeon2IMU(RobotMap.GYRO, Constants.CANBUS);
        SwerveConfiguration config = new SwerveConfiguration();

        config.drives = drives;
        config.steers = steers;
        config.encoders = encoders;
        config.gyro = gyro;
        config.modulePositions = Constants.MODULE_POSITIONS;
        config.translationalPIDGains = new double[]{0.6, 0.0, 0.0};
        config.rotationalPIDGains = new double[]{0.65, 0.0, 0.0};
        config.drivePIDFGains = new double[]{0.05, 0.0, 0.0, 1.0/Constants.MAX_MODULE_SPEED};
        config.steerPIDGains = new double[]{0.62, 0.0, 0.0};
        config.MAX_MODULE_SPEED = Constants.MAX_MODULE_SPEED;
        config.radius = Math.hypot(0.5794/2, 0.5794/2);
        config.numberOfModules = Constants.NUMBER_OF_MODULES;

        this.balanceController = new PIDController(0.012, 0, 0);

        this.limelightController = new PIDController(0.07, 0, 0);

        // TODO:: play around with this
        this.rotateController = new PIDController(0.334, 0, 0);

        this.swerve = Swerve.fromConfiguration(config);

        isPlaying = false;


    }

    public void control(double x, double y, double rotate) {
        if(isPlaying) {
            // MotionOfTheOcean.Executor.executeRecording(() -> DriveTrain.getInstance().toPose(Arrays.copyOfRange(SharkExecutor.getState().getAsArray(), 0, 3), 
            //                                                                                 SharkExecutor.getState().getAsArray()[3],
            //                                                                                 SharkExecutor.getState().getAsArray()[4]));
        }
        else if(!driveTrainLock) swerve.controlWithPercent(x, y, rotate);
    }

    public void reset() {
        swerve.reset();
    }

    public void toPose(double[] target, double linearVelocity, double angularVelocity, double heading) {
        swerve.setSwerveState(target, linearVelocity, angularVelocity, heading);
    }

    public void toPose(double[] targetData) {
        swerve.setSwerveState(new double[]{targetData[0], targetData[1], targetData[2]}, targetData[3], targetData[4], targetData[5]);
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
        return gyro.getRoll() > Constants.DRIVETRAIN_TILT_THRESHOLD || gyro.getRoll() < -9;
    }

    public boolean isChassisStable() {
        return Math.abs(gyro.getRoll()) < 4.5;
    }

    public double getGyroPitch() {
        return gyro.getRoll();
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
      SmartDashboard.putNumber("gyro tilt", getGyroPitch());

      SmartDashboard.putNumber("speed in t/s", saved.getVelocity());
      SmartDashboard.putNumber("speed in t/100ms", saved.getVelocityInTicks());
      double[] pose = swerve.getSwerveState();
        SmartDashboard.putNumber("x", pose[0]);
        SmartDashboard.putNumber("y", pose[1]);
        SmartDashboard.putNumber("rotate", pose[2]);
        SmartDashboard.putNumber("linear velocity", pose[3]);
        SmartDashboard.putNumber("heading", pose[4]);

        SmartDashboard.putBoolean("atSetpoint", swerve.atSetpoint());

        SmartDashboard.putNumber("output percent", saved.getMotorController().getMotorOutputPercent());
    }
}

