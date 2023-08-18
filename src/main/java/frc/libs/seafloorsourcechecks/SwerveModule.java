package frc.libs.seafloorsourcechecks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.libs.electronics.encoders.ThreadedEncoder;
import frc.libs.electronics.motors.LazyMotorController;
import frc.libs.electronics.motors.LazyTalonFX;
import frc.robot.Constants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private LazyTalonFX steer;
    private LazyTalonFX drive;
    private ThreadedEncoder<?> steerCoder;

    private Translation2d module_location;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

    // PIDController turn_controller = new PIDController(0.2, 0, 0);

    private static final double MODULE_GEAR_RATIO = 1 / 6.75;

    public SwerveModule(int moduleNumber, LazyTalonFX drive, LazyTalonFX steer, ThreadedEncoder<?> steerCoder, Translation2d moduleLocation) {
        this.moduleNumber = moduleNumber;

        this.angleOffset = new Rotation2d(steerCoder.getOffsetPosition());

        this.drive = drive;
        this.steer = steer;

        this.steerCoder = steerCoder;

        this.module_location = moduleLocation;

        lastAngle = getState().angle;

        // turn_controller.enableContinuousInput(Math.toRadians(0), Math.toRadians(359));
    }

    // public SwerveModule(int moduleNumber) {
    //     this.moduleNumber = moduleNumber;

    //     this.drive = new LazyTalonFX(moduleNumber, Constants.TICKS_PER_METER);
    //     this.steer = new LazyTalonFX(moduleNumber+4, Constants.TICKS_PER_METER);
    //     // this.steerCoder = new LazyTalo
    // }

    public SwerveModulePosition getState(){
        return new SwerveModulePosition(
            drive.getVelocity(), 
            getAngle()
        ); 
    }

    public Translation2d getModuleLocation() { return this.module_location; }

    public Rotation2d getAngle(){
        // return Rotation2d.fromRadians(steerCoder.getRawPosition());
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(steer.getSelectedSensorPosition(), MODULE_GEAR_RATIO));

    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (5 * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        steer.setPosition(Conversions.degreesToFalcon(angle.getDegrees(), MODULE_GEAR_RATIO));
        
        // steer.set(turn_controller.calculate(getAngle().getRadians(), angle.getRadians()));
        lastAngle = angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / 5;
            drive.set(percentOutput);
        }
        else {
            double velocity = desiredState.speedMetersPerSecond;
            drive.setVelocityInMeters(velocity);
        }
    }


    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(steerCoder.getRawPosition());
    }

    public void resetToAbsolute(){
        // double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), MODULE_GEAR_RATIO);
        // double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees())

        steer.setSelectedSensorPosition(0);
    }

}
