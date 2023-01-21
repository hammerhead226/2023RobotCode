// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.libs.wrappers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.controller.PIDController;

/**
 * @author Anish Chandra
 * Allows the Generic Use of a Motor
 */
public class GenericMotor {
    //supports falcons, talons, sparks, victors
    private TalonFX falcon;
    private CANSparkMax spark;
    private TalonSRX talon;
    private VictorSPX victor;

    private enum MotorType {
        FALCON,
        SPARK,
        TALON,
        VICTOR
    }

    public static enum PassiveMode {
        COAST,
        BRAKE
    }

    private MotorType motorType;

    private double lastSpeed;
    private double lastSensorPose;

    public GenericMotor(TalonFX falcon) {
        this.falcon = falcon;
        motorType = MotorType.FALCON;
        this.lastSpeed = 0;
        this.lastSensorPose = falcon.getSelectedSensorPosition();

    }

    public GenericMotor(CANSparkMax spark) {
        this.spark = spark;
        motorType = MotorType.SPARK;
        spark.getEncoder().setPositionConversionFactor(42);
        this.lastSpeed = 0;
        this.lastSensorPose = spark.getEncoder().getPosition();
    }

    public GenericMotor(TalonSRX talon) {
        this.talon = talon;
        motorType = MotorType.TALON;
        this.lastSpeed = 0;
        this.lastSensorPose = talon.getSelectedSensorPosition();
    }

    public GenericMotor(VictorSPX victor) {
        this.victor = victor;
        motorType = MotorType.VICTOR;    
        this.lastSpeed = 0;
        this.lastSensorPose = victor.getSelectedSensorPosition();
    }

    public void set(double speed) {
        if(speed != lastSpeed) {
            switch(motorType) {
                case FALCON:
                    falcon.set(ControlMode.PercentOutput, speed);
                    break;
                case SPARK:
                    spark.set(speed);
                    break;
                case TALON:
                    talon.set(ControlMode.PercentOutput, speed);
                    break;
                case VICTOR:
                    victor.set(ControlMode.PercentOutput, speed);
                    break;
                default:
                    break;

            }
            lastSpeed = speed;
        }
    }

    public double getSensorPose() {
        switch(motorType) {
            case FALCON:
                return falcon.getSelectedSensorPosition();
            case SPARK:
                return spark.getEncoder().getPosition();
            case TALON:
                return talon.getSelectedSensorPosition();
            case VICTOR:
                return victor.getSelectedSensorPosition();
            default:
                return -1;
        }
    }
    
    public double getVelocity() {
        switch(motorType) {
            case FALCON:
                return falcon.getSelectedSensorVelocity();
            case SPARK:
                return spark.getEncoder().getVelocity();
            case TALON:
                return talon.getSelectedSensorVelocity();
            case VICTOR:
                return victor.getSelectedSensorVelocity();
            default:
                return -1;
        }
    }

    public double getConversionFactor() {
        return spark.getEncoder().getCountsPerRevolution();
    }

    public double getSensorErr() {//automatically updates the lastPose through the set method in swerve module
        double err;
        switch(motorType) {
            case FALCON:
                err = falcon.getSelectedSensorPosition() - lastSensorPose;
                lastSensorPose = falcon.getSelectedSensorPosition();
            case SPARK:
                err = spark.getEncoder().getPosition() - lastSensorPose;
                lastSensorPose = spark.getEncoder().getPosition();
            case TALON:
                err = talon.getSelectedSensorPosition() - lastSensorPose;
                lastSensorPose = talon.getSelectedSensorPosition();
            case VICTOR:
                err = victor.getSelectedSensorPosition() - lastSensorPose;
                lastSensorPose = victor.getSelectedSensorPosition();
            default:
                err = -1;
        }
        return err;
    }

    public void inverted(boolean invert) {
        switch(motorType) {
            case FALCON:
                falcon.setInverted(invert);
                break;
            case SPARK:
                spark.setInverted(invert);
                break;
            case TALON:
                talon.setInverted(invert);
                break;
            case VICTOR:
                victor.setInverted(invert);
                break;
            default:
                break;
        }
    }

    public void setSensorPose(double sensorPos) {
        switch(motorType) {
            case FALCON:
                falcon.setSelectedSensorPosition(sensorPos);
                break;
            case SPARK:
                spark.getEncoder().setPosition(sensorPos);
            case TALON:
                talon.setSelectedSensorPosition(sensorPos);
                break;
            case VICTOR:
                victor.setSelectedSensorPosition(sensorPos);
                break;
            default:
                break;
        }
    }

    public void setNeutralMode(PassiveMode neutralMode) {
        switch(motorType) {
            case FALCON:
                if(neutralMode.equals(PassiveMode.BRAKE)) {
                    falcon.setNeutralMode(NeutralMode.Brake);
                }
                else if(neutralMode.equals(PassiveMode.COAST)) {
                    falcon.setNeutralMode(NeutralMode.Coast);
                }
                break;
            case SPARK:
                if(neutralMode.equals(PassiveMode.BRAKE)) {
                    spark.setIdleMode(IdleMode.kBrake);
                }
                else if(neutralMode.equals(PassiveMode.COAST)) {
                    spark.setIdleMode(IdleMode.kCoast);
                }
                break;
            case TALON:
                if(neutralMode.equals(PassiveMode.BRAKE)) {
                    talon.setNeutralMode(NeutralMode.Brake);
                }
                else if(neutralMode.equals(PassiveMode.COAST)) {
                    talon.setNeutralMode(NeutralMode.Coast);
                }
                break;
            case VICTOR:
                if(neutralMode.equals(PassiveMode.BRAKE)) {
                    victor.setNeutralMode(NeutralMode.Brake);
                }
                else if(neutralMode.equals(PassiveMode.COAST)) {
                    victor.setNeutralMode(NeutralMode.Coast);
                }
                break;
            default:
                break;

        }
    }

    public void configFalcon(TalonFXConfiguration config) {
        falcon.configAllSettings(config);
    }

    public void configTalon(TalonSRXConfiguration config) {
        talon.configAllSettings(config);
    }

    public void configVictor(VictorSPXConfiguration config) {
        victor.configAllSettings(config);
    }
}
