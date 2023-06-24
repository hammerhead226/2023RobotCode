package frc.libs.swerveyshark;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.libs.wrappers.PIDController;
import frc.libs.electronics.motors.LazyMotorController;
import frc.libs.electronics.encoders.ThreadedEncoder;

public class SwerveModule {
    private final LazyMotorController<?> drive;
    private final LazyMotorController<?> steer;

    private final ThreadedEncoder<?> steercoder;

    private final double kVelocity;
    private final PIDController driveController;
    private final PIDController steerController;

    private double x, y;
    private double lastDrivePosition;

    private final double[] defaultModulePosition;

    public SwerveModule(LazyMotorController<?> drive,
                        LazyMotorController<?> steer,
                        ThreadedEncoder<?> encoder,
                        double[] modulePosition,
                        double[] drivePIDFGains,
                        double[] steerPIDGains) {
        this.drive = drive;
        this.steer = steer;
        this.steercoder = encoder;
        this.defaultModulePosition = modulePosition;
        this.driveController = new PIDController(drivePIDFGains);
        this.steerController = new PIDController(steerPIDGains);
        this.kVelocity = drivePIDFGains[3];

        x = defaultModulePosition[0];
        y = defaultModulePosition[1];

        lastDrivePosition = drive.getPosition();
    }

    public void set(double velocity, double adjustedTargetAngle, double chassisHeading) {
        updateModulePosition(chassisHeading);

        double[] errorAndVelocityMultiplier = optimizeErrorAndVelocityMultiplier(steercoder.getContinuousPosition(), adjustedTargetAngle);
        double error = errorAndVelocityMultiplier[0];
        double velocityMultiplier = errorAndVelocityMultiplier[1];

        velocity *= velocityMultiplier;

        double steerSpeed = steerController.calculate(error);

        SmartDashboard.putNumber("velocity", velocity);
        SmartDashboard.putNumber("targetAngle", adjustedTargetAngle);
        SmartDashboard.putNumber("steercoder", steercoder.getContinuousPosition());
        SmartDashboard.putNumber("steer", steerSpeed);

        drive.setVelocityInMeters(velocity);
        steer.set(steerSpeed);
    }

    private double[] optimizeErrorAndVelocityMultiplier(double current, double target) {
        double err = (target - current) % (2 * Math.PI);

        if(err > Math.PI) err -= 2 * Math.PI;
        else if(err < -Math.PI) err += 2 * Math.PI;

        double velocityMultiplier = -1;

        if(err > Math.PI/2) err -= Math.PI;
        else if(err < -Math.PI/2) err += Math.PI;
        else velocityMultiplier = 1;

        return new double[]{err, velocityMultiplier};
    }

    private void updateModulePosition(double currentChassisHeading) {
        double distance = drive.getPosition() - lastDrivePosition;
        double direction = steercoder.getContinuousPosition() % (2 * Math.PI);

        x += distance * Math.cos(direction + currentChassisHeading);
        y += distance * Math.sin(direction + currentChassisHeading);
        lastDrivePosition = drive.getPosition();
    }

    public double[] getVelocityVector() {
        double xVelocity = drive.getVelocity() * Math.cos(steercoder.getContinuousPosition() % (2 * Math.PI));
        double yVelocity = drive.getVelocity() * Math.sin(steercoder.getContinuousPosition() % (2 * Math.PI));
        return new double[]{xVelocity, yVelocity};
    }

    public double[] getPose() {
        return new double[]{x, y};
    }

    public double getModuleRotationalPose() {
        return steercoder.getOffsetPosition();
    }

    public void reset() {
        x = defaultModulePosition[0];
        y = defaultModulePosition[1];
    }
}
