package frc.libs.electronics.motors;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LazyTalonFX implements LazyMotorController<TalonFX> {

    private TalonFX motor;
    private double lastSpeed = 0.;
    private double lastPosition = 0.;

    private double ticksPerMeter;

    public LazyTalonFX(TalonFX motor, double ticksPerMeter) {
        this.motor = motor;
        this.ticksPerMeter = ticksPerMeter;
    }

    public LazyTalonFX(int port, double ticksPerMeter) {
        this.motor = new TalonFX(port);
        this.ticksPerMeter = ticksPerMeter;
    }

    public void applyConfiguration(TalonFXConfiguration config) {
        this.motor.configAllSettings(config);
    }

    @Override
    public TalonFX getMotorController() {
        return motor;
    }

    @Override
    public void set(double speed) {
        if(speed != lastSpeed)
            motor.set(ControlMode.PercentOutput, speed);
        lastSpeed = speed;
    }

    public void setPosition(double position) {
        if (position != lastSpeed) motor.set(ControlMode.Position, position); lastPosition = position;
    }

    @Override
    public void setVelocityInMeters(double speed) {
        speed *= ticksPerMeter;
        speed /= 10;
        motor.set(ControlMode.Velocity, speed);
    }

    @Override
    public double getPosition() {
        return motor.getSelectedSensorPosition()/ticksPerMeter;
    }

    public double getTicks() {
        return motor.getSelectedSensorPosition();
    }

    public double getVelocityInTicks() {
        return motor.getSelectedSensorVelocity();
    }

    @Override
    public double getVelocity() {
        return (motor.getSelectedSensorVelocity() * 10)/(ticksPerMeter);
    }

    public void setSelectedSensorPosition(double absolutePosition) {
        motor.setSelectedSensorPosition(absolutePosition);
    }

    public double getSelectedSensorPosition() {
        return motor.getSelectedSensorPosition();
    }

    public void configAllSetting(TalonFXConfiguration config) {
        motor.configAllSettings(config);
    }
}
