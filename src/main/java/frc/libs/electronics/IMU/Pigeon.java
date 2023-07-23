package frc.libs.electronics.IMU;

import com.ctre.phoenix.sensors.PigeonIMU;

public class Pigeon implements Gyro<PigeonIMU> {

    private PigeonIMU gyro;

    public Pigeon(int port) {
        gyro = new PigeonIMU(port);
    }

    @Override
    public PigeonIMU getGyro() {
        // TODO Auto-generated method stub
        return gyro;
    }

    @Override
    public double getPitch() {
        // TODO Auto-generated method stub
        return gyro.getPitch();
    }

    @Override
    public double getRoll() {
        // TODO Auto-generated method stub
        return gyro.getRoll();
    }

    @Override
    public double getYaw() {
        // TODO Auto-generated method stub
        return gyro.getYaw();
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        gyro.setYaw(0);
        
    }

    @Override
    public void resetFlip() {
        // TODO Auto-generated method stub
        gyro.setYaw(180);
        
    }

    @Override
    public void setYaw(double value) {
        // TODO Auto-generated method stub
        gyro.setYaw(value);
        
    }

}