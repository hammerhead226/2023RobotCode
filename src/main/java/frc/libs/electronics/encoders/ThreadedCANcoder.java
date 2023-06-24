package frc.libs.electronics.encoders;

import com.ctre.phoenix.sensors.CANCoder;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class ThreadedCANcoder implements ThreadedEncoder<CANCoder> {

    private final CANCoder encoder;

    private final ScheduledExecutorService threadService;

    private final double overflowThreshold;
    private final double offset;

    private int overflows;
    private double lastSensorPose;


    public ThreadedCANcoder(int port, double threshold, double offset, int delayms, String bus) {
        encoder = new CANCoder(port, bus);
        this.overflowThreshold = threshold;
        this.offset = offset;

        overflows = 0;
        lastSensorPose = encoder.getAbsolutePosition();

        threadService = Executors.newSingleThreadScheduledExecutor();
        threadService.scheduleAtFixedRate(this::trackOverflows, 0, delayms, TimeUnit.MILLISECONDS);
    }

    @Override
    public synchronized void trackOverflows() {
        double err = getRawPosition() - lastSensorPose;
        lastSensorPose = getRawPosition();
        if(err > overflowThreshold) overflows++;
        else if(err < -overflowThreshold) overflows--;
    }

    @Override
    public synchronized double getRawPosition() {
        return Math.toRadians(encoder.getAbsolutePosition());
    }

    @Override
    public synchronized double getOffsetPosition() {
        return -(overflows * 360 + encoder.getAbsolutePosition() + offset);
    }

    @Override
    public synchronized double getContinuousPosition() {
        return Math.toRadians(overflows * 360 + getOffsetPosition());
    }



    @Override
    public synchronized CANCoder getEncoder() {
        return encoder;
    }
}
