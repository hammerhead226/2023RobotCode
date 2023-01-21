// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.libs.wrappers;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.AnalogInput;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * <p> Wraps AnalogInput Object and CANCoder Object to Simplify Position Data Collection </p>
 * @author Anish Chandra
*/
public class GenericEncoder {

    private AnalogInput analogInput;
    private CANCoder canCoder;

    private enum EncoderType {
        ANALOG,
        CANCODER
    }

    private EncoderType encoderType;

    private Object encoderLock;
    private ScheduledExecutorService service;
    
    private int overflows;
    private double lastSensorPose;
    private double moduleOffset;

    private int ticksPerRotation;
    private int threshold;

    private double threshold_rad;

    public GenericEncoder(AnalogInput analogInput, int ticksPerRotation, int threshold, int moduleOffset) {
        this.analogInput = analogInput;
        this.ticksPerRotation = ticksPerRotation;
        this.threshold = threshold;
        this.moduleOffset = moduleOffset;
        encoderType = EncoderType.ANALOG;
        encoderLock = new Object();
        lastSensorPose = analogInput.getValue();
        overflows = 0;
        service = Executors.newSingleThreadScheduledExecutor();
        service.scheduleAtFixedRate(this::trackOverflows, 0, 10, TimeUnit.MILLISECONDS);
    }

    public GenericEncoder(CANCoder canCoder, double threshold_rad, double moduleOffset) {
        this.canCoder = canCoder;
        this.threshold_rad = threshold_rad;
        this.moduleOffset = moduleOffset;
        encoderType = EncoderType.CANCODER;
        encoderLock = new Object();
        lastSensorPose = 0;
        overflows = 0;
        service = Executors.newSingleThreadScheduledExecutor();
        service.scheduleAtFixedRate(this::trackOverflows, 0, 10, TimeUnit.MILLISECONDS);
    }

    private void trackOverflows() {
        synchronized(encoderLock) {
            double err;
            switch(encoderType) {
                case ANALOG:
                    err = analogInput.getValue() - lastSensorPose;
                    lastSensorPose = analogInput.getValue();
                    if(err > threshold) overflows++;
                    else if(err < -threshold) overflows--;
                    break;
                case CANCODER:
                    err = Math.toRadians(canCoder.getAbsolutePosition()) - lastSensorPose;
                    lastSensorPose = Math.toRadians(canCoder.getAbsolutePosition());
                    if(err > threshold_rad) overflows--;
                    else if(err < -threshold_rad) overflows++;
                    break;
                default:
                    break;

            }
        }
    }

    public double getAbsolutePosition() {
        synchronized(encoderLock) {
            switch(encoderType) {
                case ANALOG:
                    return -analogInput.getValue() + moduleOffset;
                case CANCODER:
                    return canCoder.getAbsolutePosition() + moduleOffset;
                default:
                    return -1;

            }
        }
    }

    public double getContinuousPosition() {
        synchronized(encoderLock) {
            switch(encoderType) {
                case ANALOG:
                    return ticksToRadians(overflows * ticksPerRotation + getAbsolutePosition());
                case CANCODER:
                    return Math.toRadians(overflows * 360 + getAbsolutePosition());
                default:
                    return -1;

            }
        }
    }

    public double getModuleOffset() {
        synchronized(encoderLock) {
            switch(encoderType) {
                case ANALOG:
                    return overflows * ticksPerRotation + getAbsolutePosition();
                
                case CANCODER:
                    return -(overflows * 360 + getAbsolutePosition());
                default:
                    return -1;
                
            }
        }
    }

    private double ticksToRadians(double ticks) {
        return ticks * 2 * Math.PI / ticksPerRotation;
    }    
}