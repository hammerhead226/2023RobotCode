package frc.libs.riptide;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.libs.electronics.IMU.Gyro;
import frc.libs.electronics.IMU.Pigeon2IMU;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class SwerveDrive {
    
    public static double kMaxDriveSpeed; // meters per second

    // public static double kMaxAngularSpeed; // rotations per second

    // public static double kRadius;

    // public static double kEncoderResolution;

    private SwerveModule m_frontLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_backLeft;
    private SwerveModule m_backRight;
    
    private Gyro<?> m_gyro;
    
    private SwerveDriveKinematics m_kinematics;
    
    private SwerveDriveOdometry m_odometry;
    
    
    public SwerveDrive(Gyro<?> gyro, double maxdrivespeed, SwerveModule fl, SwerveModule fr, SwerveModule bl, SwerveModule br) {
        this.m_gyro = gyro;
        kMaxDriveSpeed = maxdrivespeed;
        // kMaxAngularSpeed = maxrotationspeed;
        // kRadius = radius;
        // kEncoderResolution = resolution;
        m_frontLeft = fl;
        m_frontRight  = fr;
        m_backLeft = bl;
        m_backRight = br;
        
        m_gyro.reset();
        
        m_kinematics =
        new SwerveDriveKinematics(
            m_frontLeft.location, m_frontRight.location, m_backLeft.location, m_backRight.location);
            
        m_odometry =
        new SwerveDriveOdometry(
            m_kinematics,
            new Rotation2d(m_gyro.getYaw()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }

        );
                
    }
    public Rotation2d getRotation2d() {return new Rotation2d(m_gyro.getYaw());}
    
    /** Returns current position of the drivetrain, units in meters */ 
    public Pose2d getPose() { return m_odometry.getPoseMeters(); }

    /** Returns SwerveDriveKinematics Object */
    public SwerveDriveKinematics getKinematics() { return m_kinematics; }
    
    /** Updates the field relative position of the robot. 
     *  Call once per periodic
     */
    public void updateOdometry() {
        m_odometry.update(
            getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }
        );
    }


    /**
   * Method to drive the robot using joystick info.
   *
   * @param vx Speed of the robot in the x direction (forward).
   * @param vy Speed of the robot in the y direction (sideways).
   * @param theta Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
    public void control(double vx, double vy, double theta, boolean fieldRelative) {
        var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, theta, getRotation2d()) 
                    : new ChassisSpeeds(vx, vy, theta));

        setModuleStates(swerveModuleStates);
    } 

    public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxDriveSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);

        
    }

}
