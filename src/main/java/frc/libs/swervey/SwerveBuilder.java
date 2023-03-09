package frc.libs.swervey;

import frc.libs.wrappers.GenericEncoder;
import frc.libs.wrappers.GenericMotor;
import frc.libs.wrappers.Gyro;

/** 
 * <p> Builds the Swerve Object </p>
 * @author Anish Chandra
*/
public class SwerveBuilder {
    private GenericMotor[] drives;
    private GenericMotor[] steers;
    private GenericEncoder[] encoders;
    private Gyro gyro;

    private double[][] modulePositions;
    private double[][] pidGains;
    private double[][] scheduledGains;
    private double[] thresholds;

    int numberOfModules;

    double[] speedBounds;
    double[] accelerationParameters;

    double ticksPerFeet;
    double[] allowedErrors;
    private double velocityFeedForward;

    boolean[] verfiedParts;
    int numberOfParameters = 5;
    

    public SwerveBuilder(GenericMotor[] drives, GenericMotor[] steers, GenericEncoder[] encoders, Gyro gyro) {
        this.drives = drives;
        this.steers = steers;
        this.encoders = encoders;
        this.gyro = gyro;
        verfiedParts = new boolean[numberOfParameters];
        this.numberOfModules = drives.length;
    }

    public Swerve buildSwerve() throws NullPointerException {
        for(boolean boo : verfiedParts) {
            if(!boo) {
                throw new NullPointerException("Missing Swerve Parameters");
            }
        }

        Swerve swerve = new Swerve(drives, steers, encoders, gyro, modulePositions, numberOfModules);

        swerve.configureDrivePIDGains(pidGains[0]);
        swerve.configureSteerPIDGains(scheduledGains[0], pidGains[1]);
        swerve.configureRotatePIDGains(scheduledGains[1], pidGains[2]);

        swerve.configureThresholds(thresholds[0], thresholds[1], thresholds[2]);

        swerve.configureAutonomousParameters(ticksPerFeet, allowedErrors[0], allowedErrors[1], velocityFeedForward);

        swerve.configureSpeeds(speedBounds[0], speedBounds[1]);
        swerve.configureAccelerationParameters(accelerationParameters[0], accelerationParameters[1]);

        return swerve;
    }

    public SwerveBuilder PIDGains(double[][] pidGains, double[][] scheduledGains, double[] thresholds) {
        this.pidGains = pidGains;
        this.scheduledGains = scheduledGains;
        this.thresholds = thresholds;
        verfiedParts[0] = true;
        return this;
    }

    public SwerveBuilder modulePositions(double[][] modPoses) {
        this.modulePositions = modPoses;
        verfiedParts[1] = true;
        return this;
    }

    public SwerveBuilder speedBounds(double[] speedBounds) {
        this.speedBounds = speedBounds;
        verfiedParts[2] = true;
        return this;
    }

    public SwerveBuilder accelerationParameters(double[] accelerationParameters) {
        this.accelerationParameters = accelerationParameters;
        verfiedParts[3] = true;
        return this;
    }

    public SwerveBuilder autonomousParameters(double ticksPerFeet, double[] allowedErrors, double velocityFeedForwardGain) {
        this.ticksPerFeet = ticksPerFeet;
        this.allowedErrors = allowedErrors;
        this.velocityFeedForward = velocityFeedForwardGain;
        verfiedParts[4] = true;
        return this;
    }    
}