package frc.libs.electronics.IMU;

public interface Gyro<T> {

    public T getGyro();

    public double getPitch();

    public double getRoll();

    public double getYaw();

    public void setYaw(double value);

    public void reset();
}
