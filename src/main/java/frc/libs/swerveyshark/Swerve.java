package frc.libs.swerveyshark;

import frc.libs.wrappers.PIDController;
import frc.libs.electronics.IMU.Gyro;
import frc.libs.electronics.motors.LazyMotorController;
import frc.libs.electronics.encoders.ThreadedEncoder;

import java.util.Arrays;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;


public class Swerve {
    private final SwerveModule[] modules;


    private Gyro<?> gyro;

    private final double[] rotationAngles;
    private double maxModuleSpeed;

    private boolean highSpeedMode;

    private double highPercentSpeed;
    private double lowPercentSpeed;
    private double currentPercentSpeed;

    private double accelerationRate;
    private double initialSpeed;

    private double allowedTranslationalError;
    private double allowedRotationalError;

    private double[] speeds;
    private final double[] thetas;

    private final PIDController translationalPIDController;
    private final PIDController rotationalPIDController;

    private final double radius;

    private double[] target;

    private boolean isFlipped;

    public Swerve(
            LazyMotorController<?>[] drives,
            LazyMotorController<?>[] steers,
            ThreadedEncoder<?>[] steercoders,
            Gyro<?> gyro,
            double[][] modulePositions,
            double[] translationalPIDGains,
            double[] rotationalPIDGains,
            double[] drivePIDFGains,
            double[] steerPIDGains,
            double MAX_MODULE_SPEED,
            double radius,
            int numberOfModules) {

        modules = new SwerveModule[numberOfModules];

        speeds = new double[numberOfModules];
        thetas = new double[numberOfModules];

        maxModuleSpeed = MAX_MODULE_SPEED;

        rotationAngles = new double[numberOfModules];

        translationalPIDController = new PIDController(translationalPIDGains);
        rotationalPIDController = new PIDController(rotationalPIDGains);

        this.radius = radius;

        this.gyro = gyro;

        this.target = new double[3];
        
        this.isFlipped = false;
        
        for(int i = 0; i < numberOfModules; i++) {
            modules[i] = new SwerveModule(
                    drives[i],
                    steers[i],
                    steercoders[i],
                    modulePositions[i],
                    drivePIDFGains,
                    steerPIDGains);

            speeds[i] = 0;
            thetas[i] = 0;

            rotationAngles[i] = Math.atan2(modulePositions[i][1], modulePositions[i][0]) + (Math.PI/2);
        }
    }

    public static Swerve fromConfiguration(SwerveConfiguration conf) {
        if(!conf.isConfigReady())
            throw new RuntimeException("Configuration is incomplete! Make sure all parameters are filled");
        return new Swerve(
                conf.drives,
                conf.steers,
                conf.encoders,
                conf.gyro,
                conf.modulePositions,
                conf.translationalPIDGains,
                conf.rotationalPIDGains,
                conf.drivePIDFGains,
                conf.steerPIDGains,
                conf.MAX_MODULE_SPEED,
                conf.radius,
                conf.numberOfModules
        );
    }

    public void changeMaxVelocity(double mV) {
        this.maxModuleSpeed = mV;
    }

    //this is the controller inputed
    public void controlWithPercent(double x, double y, double rotate) {
        control(x * maxModuleSpeed, y*maxModuleSpeed, -rotate*maxModuleSpeed);
    }

    //assume all vectors given are m/s
    private void control(double x, double y, double rotate) {
        double chassisHeading = gyro.getYaw();
        // double chassisHeading = 0;

        for(int i = 0; i < modules.length; i++) {
            double[] rotationVector = new double[] {
                    rotate * Math.cos(rotationAngles[i] + chassisHeading),
                    rotate * Math.sin(rotationAngles[i] + chassisHeading)
            };

            double[] targetVector = vectorSum(rotationVector, new double[]{x, y});
            double speed = targetVector[0];
            double theta = targetVector[1];

            theta -= chassisHeading;

            if(!(x == 0 && y == 0 && rotate == 0))
                thetas[i] = theta;

            speeds[i] = speed;
        }

        speeds = normalize(speeds);

        for(int i = 0; i < modules.length; i++) {
            modules[i].set(speeds[i], thetas[i], chassisHeading);
        }

    }

    public void toggleSpeed() {
        highSpeedMode = !highSpeedMode;
    }

    public void setSwerveState(double[] position, double linearVelocity, double angularVelocity, double directionalMotion) {
        double[] currentSwerveState = getSwerveState();

        double xLinearVelocity = linearVelocity * Math.cos(directionalMotion);
        double yLinearVelocity = linearVelocity * Math.sin(directionalMotion);

        double adjustedXLinearVel = xLinearVelocity * translationalPIDController.calculate(currentSwerveState[0], position[0]);
        double adjustedYLinearVel = yLinearVelocity * translationalPIDController.calculate(currentSwerveState[1], position[1]);

        // SmartDashboard.putNumber("y error/t", position[0] - currentSwerveState[0]);

        double adjustedAngularVel = angularVelocity * rotationalPIDController.calculate(currentSwerveState[2], position[2]);

        // SmartDashboard.putNumber("rotate error/t", position[2] - currentSwerveState[2]);

        // SmartDashboard.putNumber("adjustedX", adjustedXLinearVel);
        // SmartDashboard.putNumber("adjustedY", adjustedYLinearVel);
        // SmartDashboard.putNumber("adjustedR", adjustedAngularVel*radius);

        control(adjustedXLinearVel, adjustedYLinearVel, -adjustedAngularVel * radius);
    }

    public double[] getSwerveState() {
        double x = 0, y = 0;
        double xVelocity = 0, yVelocity = 0;
        for(SwerveModule mod : modules) {
            double[] modPose = mod.getPose();
            double[] modVelocity = mod.getVelocityVector();

            x += modPose[0]/modules.length;
            y += modPose[1]/modules.length;

            xVelocity += modVelocity[0]/modules.length;
            yVelocity += modVelocity[1]/modules.length;
        }

        double linearVelocity = Math.hypot(xVelocity, yVelocity);
        double heading = Math.atan2(yVelocity, xVelocity);

        return new double[]{x, y, gyro.getYaw(), linearVelocity, heading};
    } 

    public void reset() {
        for(SwerveModule mod : modules) {
            mod.reset();
        }
        gyro.reset();
        isFlipped = false;
    }

    public void resetFlip() {
        for(SwerveModule mod : modules) {
            mod.reset();
        }
        gyro.resetFlip();
        isFlipped = true;
    }

    public boolean getIsFlipped() {
        return isFlipped;
    }

    public boolean atSetpoint() {
        double[] currentPose = getSwerveState();
        double xErr = target[0] - currentPose[0];
        double yErr = target[1] - currentPose[1];
        double thetaErr = target[2] - currentPose[2];
    
        return (Math.abs(xErr) <= allowedTranslationalError) && (Math.abs(yErr) <= allowedTranslationalError) && (Math.abs(thetaErr) <= allowedRotationalError);
      }

    public void setMaxModuleSpeed(double value) {
        maxModuleSpeed = value;
    }

    public double getModuleRotationalPose(int module) {
        return modules[module].getModuleRotationalPose();
    }

    private double[] vectorSum(double[] a, double[] b) {
        double xSum = a[0] + b[0];
        double ySum = a[1] + b[1];
        return new double[]{Math.hypot(xSum, ySum), Math.atan2(ySum, xSum)};
    }

    private double[] normalize(double[] arr) {
        double inputtedMaxSpeed = Arrays.stream(arr).max().getAsDouble();
        if(inputtedMaxSpeed > maxModuleSpeed)
            for(int i = 0; i < arr.length; i++) {
                arr[i] /= inputtedMaxSpeed;
                arr[i] *= maxModuleSpeed;
            }
        return arr;
    }
}
