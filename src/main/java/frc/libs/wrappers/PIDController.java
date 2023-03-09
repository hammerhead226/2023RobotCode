package frc.libs.wrappers;

public class PIDController {
    double kP, kI, kD;

    double setpoint;

    double lastErr;

    double totalErr;


    public PIDController(double kp, double ki, double kd) {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;

        lastErr = 0;
        totalErr = 0;
    }

    public PIDController(double[] gains) {
        kP = gains[0];
        kI = gains[1];
        kD = gains[2];

        lastErr = 0;
        totalErr = 0;
    }

    public PIDController() {
        kP = 0;
        kI = 0;
        kD = 0;

        lastErr = 0;
        totalErr = 0;
    }
    
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public double calculate(double current, double setpoint) {
        double elapsed = 0.02;
        double err = setpoint - current;

        if(kI != 0)
            totalErr = clamp(totalErr + err * elapsed, -1.0/kI, 1.0/kI);

        double derivedErr = (err - lastErr)/elapsed;
        double output = kP * err + kI * totalErr + kD * derivedErr;

        lastErr = err;
        return output;
    }

    public double calculate(double measurement) {
        return calculate(measurement , 0);
    }

    public void setP(double kP) {
        this.kP = kP;
    }

    public void setI(double kI) {
        this.kI = kI;
    }

    public void setD(double kD) {
        this.kD = kD;
    }

    public void setPID(double[] gains) {
        this.kP = gains[0];
        this.kI = gains[1];
        this.kD = gains[2];
    }
}
