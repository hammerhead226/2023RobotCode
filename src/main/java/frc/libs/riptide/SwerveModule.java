package frc.libs.riptide;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.libs.electronics.encoders.ThreadedEncoder;
import frc.libs.electronics.motors.LazyMotorController;

public class SwerveModule {
    private static double kWheelRadius =  0.0762; // meters
    private static int kEncoderResolution = 4096;
    private static double kModuleMaxAngularVelocity;
    private static double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second per second
    private static double kModuleMaxLinearVelocity = 2; // meters per second

    private final LazyMotorController<?> drive;
    private final LazyMotorController<?> turn;
    private final ThreadedEncoder<?> encoder;

    final int mod_num;

    // Swerve Module Locations (+x = front, +y = left)
    public final Translation2d location;

    private final PIDController m_drivePID = new PIDController(0.1, 0.0, 0.0);
    private final PIDController m_turnPID = new PIDController(0.04, 0.0, 0.00);

    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.2, 0.1);
    // TODO:: calc this for radians/second (max displacement should be pi)
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.2, 0.1);
// 
    public SwerveModule(LazyMotorController<?> mdrive, LazyMotorController<?> mturn, ThreadedEncoder<?> m_encoder, Translation2d mlocation, double maxrotationspeed, int mod_num) {
        drive = mdrive;
        turn = mturn;
        encoder = m_encoder;
        location = mlocation;
        kModuleMaxAngularVelocity = maxrotationspeed;
        this.mod_num = mod_num;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), getEncoderAngle());
    }

    public SwerveModuleState getState() { return new SwerveModuleState(drive.getVelocity(), getEncoderAngle()); }

    private Rotation2d getEncoderAngle() { return new Rotation2d(encoder.getRawPosition() % (2 * Math.PI)); }

    private double getDriveDistance() { return drive.getPosition(); }


    public void setDesiredState(SwerveModuleState desiredState) {
        SmartDashboard.putNumber("swerve/mod_" + mod_num + "_Desired State", desiredState.angle.getDegrees());
        

        
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getState().angle);

        drive.set(desiredState.speedMetersPerSecond / kModuleMaxLinearVelocity);
        turn.set(m_turnPID.calculate(getEncoderAngle().getRadians(), desiredState.angle.getRadians()));
        SmartDashboard.putString("Module " + mod_num + " state:", state.toString());

        // final double driveOutput = m_drivePID.calculate(drive.getVelocity(), state.speedMetersPerSecond);

        // final double driveFF = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // final double turnOutput = m_turnPID.calculate(getEncoderAngle().getRadians(), state.angle.getRadians());

        // SmartDashboard.putNumber("swerve/mod_" + mod_num + "_PID_OUTPUT", turnOutput);

        // final double turnFF = m_turnFeedforward.calculate(m_turnPID.getSetpoint().velocity);

        // drive.set(driveOutput + driveFF);
        // // turn.set(turnOutput + turnFF);
        // turn.set(0);
        // drive.set(driveOutput);
        // turn.set(turnOutput);

        // SmartDashboard.putNumberArray("/swerve/desiredState", new double[]{state.speedMetersPerSecond, state.angle.getRadians()});
        // SmartDashboard.putNumber("/swerve/driveOutput", driveOutput);
        // SmartDashboard.putNumber("/swerve/driveFF", driveFF);
        // SmartDashboard.putNumber("/swerve/turnOutput", turnOutput);
        // SmartDashboard.putNumber("/swerve/turnFF", turnFF);
    }
}
