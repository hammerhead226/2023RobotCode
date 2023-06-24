package frc.libs.electronics.motors;

public interface LazyMotorController<T> {

    T getMotorController();

    void set(double speed);

    void setVelocityInMeters(double speed);

    double getPosition();

    double getVelocity();
}
