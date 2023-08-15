package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.electronics.IMU.Pigeon2IMU;
import frc.libs.electronics.encoders.ThreadedCANcoder;
import frc.libs.electronics.motors.LazyTalonFX;
import frc.libs.riptide.SwerveDrive;
import frc.libs.riptide.SwerveModule;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Swerve extends SubsystemBase {
    
    private static final Swerve INSTANCE = new Swerve();

    public static Swerve getInstance() {
        return INSTANCE;
    }

    private SwerveDrive dt;

    private Pigeon2IMU gyro;

    /**
     * 
     */
    private Swerve() {
        LazyTalonFX[] drives = new LazyTalonFX[Constants.NUMBER_OF_MODULES];
        LazyTalonFX[] steers = new LazyTalonFX[Constants.NUMBER_OF_MODULES];
        ThreadedCANcoder[] encoders = new ThreadedCANcoder[Constants.NUMBER_OF_MODULES];

        for(int i = 0; i < Constants.NUMBER_OF_MODULES; i++) {
            TalonFX drive = new TalonFX(RobotMap.DRIVE_MOTORS[i], Constants.CANBUS);
            TalonFX steer = new TalonFX(RobotMap.STEER_MOTORS[i], Constants.CANBUS);

            drive.configOpenloopRamp(0.1);
            drive.configClosedloopRamp(0.1);
            drive.setNeutralMode(NeutralMode.Brake);

            drives[i] = new LazyTalonFX(drive, Constants.TICKS_PER_METER);
            steers[i] = new LazyTalonFX(steer, Constants.TICKS_PER_METER);

            encoders[i] = new ThreadedCANcoder(i, Math.PI, Constants.MODULE_OFFSETS[i], 10, Constants.CANBUS, false);
        }

        final double MODULE_POSITION_OFFSET = 0.381;
        final double MODULE_MAX_ANGULAR_VELOCITY = 2 * Math.PI;
        SwerveModule _fl = new SwerveModule(drives[0], steers[0], encoders[0], new Translation2d(MODULE_POSITION_OFFSET, MODULE_POSITION_OFFSET), MODULE_MAX_ANGULAR_VELOCITY);
        SwerveModule _fr = new SwerveModule(drives[1], steers[1], encoders[1], new Translation2d(MODULE_POSITION_OFFSET, -MODULE_POSITION_OFFSET), MODULE_MAX_ANGULAR_VELOCITY);
        SwerveModule _bl = new SwerveModule(drives[2], steers[2], encoders[2], new Translation2d(-MODULE_POSITION_OFFSET, MODULE_POSITION_OFFSET), MODULE_MAX_ANGULAR_VELOCITY);
        SwerveModule _br = new SwerveModule(drives[3], steers[3], encoders[3], new Translation2d(-MODULE_POSITION_OFFSET, -MODULE_POSITION_OFFSET), MODULE_MAX_ANGULAR_VELOCITY);
        
        gyro = new Pigeon2IMU(RobotMap.GYRO, Constants.CANBUS);



        dt = new SwerveDrive(gyro, 1, _fl, _fr, _bl, _br);

    }

    public void control(double x, double y, double theta) {
        dt.control(x, y, theta, true);
    }

    public void updateOdometry() {
        dt.updateOdometry();
    }

}
