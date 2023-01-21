// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.libs.wrappers;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**  
 * @author Anish Chandra
 * Allows the Generic Usage of a Pigeon IMU
*/
public class Gyro {
    private PigeonIMU pigeon;
    public Gyro(TalonSRX controller) {
        pigeon = new PigeonIMU(controller);
    }

    public Gyro(int port) {
        pigeon = new PigeonIMU(port);
    }

    public double getYaw() {
        // return Math.toRadians(pigeon.getFusedHeading());
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return Math.toRadians(ypr[0]);
    }

    public void zeroGyro() {
        // pigeon.setFusedHeading(0);
        pigeon.setYaw(0);
    }

}
