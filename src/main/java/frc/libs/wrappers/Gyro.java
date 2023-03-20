// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.libs.wrappers;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**  
 * @author Anish Chandra
 * Allows the Generic Usage of a Pigeon IMU
*/
public class Gyro {
    private PigeonIMU pigeon;
    private Pigeon2 pigeon2;

    private enum PIGEON_VERSION {
        IMU,
        TWO
    }

    private PIGEON_VERSION version;

    public Gyro(TalonSRX controller) {
        pigeon = new PigeonIMU(controller);
        version = PIGEON_VERSION.IMU;
    }

    public Gyro(int port) {
        pigeon2 = new Pigeon2(port);
        version = PIGEON_VERSION.TWO;
    }

    public Gyro(int port, String canbus) {
        pigeon2 = new Pigeon2(port, canbus);
        version = PIGEON_VERSION.TWO;
    }

    

    public double getYaw() {
        // return Math.toRadians(pigeon.getFusedHeading());
        double[] ypr = new double[3];
        pigeon2.getYawPitchRoll(ypr);
        return Math.toRadians(ypr[0]);
    }

    public double getTilt() {
        double[] ypr = new double[3];
        return pigeon2.getPitch();
    }

    public void zeroGyro() {
        // pigeon.setFusedHeading(0);
        pigeon2.setYaw(0);
    }

}
