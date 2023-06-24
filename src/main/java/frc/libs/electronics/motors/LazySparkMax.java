package frc.libs.electronics.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.libs.wrappers.PIDController;;;


public class LazySparkMax implements LazyMotorController<CANSparkMax> {

    private CANSparkMax motor;

    private double lastSpeed = 0.;

    private double ticksPerMeter;

    private PIDController controller;

    private double feedforward;

    public LazySparkMax(CANSparkMax motor, double ticksPerMeter) {
        this.motor = motor;
        this.ticksPerMeter = ticksPerMeter;

        controller = new PIDController();
    }

    public LazySparkMax(int port, MotorType type, double ticksPerMeter) {
        this.motor = new CANSparkMax(port, type);
        this.ticksPerMeter = ticksPerMeter;

        controller = new PIDController();
    }

    public LazySparkMax(int port, MotorType type, double ticksPerMeter, double[] gains, double feedforward) {
        this.motor = new CANSparkMax(port, type);
        this.ticksPerMeter = ticksPerMeter;

        controller = new PIDController(gains);
        this.feedforward = feedforward;
    }

    @Override
    public CANSparkMax getMotorController() {
        return motor;
    }

    @Override
    public void set(double speed) {
        if(speed != lastSpeed)
            motor.set(speed);
        lastSpeed = speed;
    }

    @Override
    public void setVelocityInMeters(double speed) {
        motor.set(
                feedforward * speed + controller.calculate(getVelocity(), speed)
                );
    }

    @Override
    public double getPosition() {
        return motor.getEncoder().getPosition()/ticksPerMeter;
    }

    @Override
    public double getVelocity() {
        return motor.getEncoder().getVelocity()/ticksPerMeter;
    }
}
