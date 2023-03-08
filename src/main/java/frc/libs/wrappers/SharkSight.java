package frc.libs.wrappers;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SharkSight {
    private static NetworkTable jetson = NetworkTableInstance.getDefault().getTable("SharkSight");
    
    private static ObjectMapper objectMapper = new ObjectMapper();

    private static Detect intakeDetect;
    private static Detect gripperDetect;

    public static double getNetworkFPS() {
        return jetson.getEntry("Network FPS").getDouble(0.0);
    }

    public static double getPiplineFPS() {
        return jetson.getEntry("Pipeline FPS").getDouble(0.0);
    }

    public static double getLatency() {
        return jetson.getEntry("Latency").getDouble(0.0);
    }

    public static String getClosestIntakeDetection() {
        return jetson.getEntry("Intake Closest Detection").getString("");
    }

    public static String getClosestGripperDetection() {
        return jetson.getEntry("Gripper Closest Detection").getString("");
    }

    public static Detect getClosestIntakeDetect() {
        return intakeDetect;
    }

    public static Detect getClosestGripperDetect() {
        return gripperDetect;
    }

    public static String getConeRotation() {
        return jetson.getEntry("Cone Rotation").getString("");
    }

    public static double getCpuTemp() {
        return jetson.getEntry("CPU Temp").getDouble(0);
    }

    public static double getGpuTemp() {
        return jetson.getEntry("GPU Temp").getDouble(0);
    }

    public static void shutdown() {
        jetson.getEntry("Shutdown").setBoolean(true);
    }

    public static void enable() {
        jetson.getEntry("Enabled").setBoolean(true);
    }
    
    public static void disable() {
        jetson.getEntry("Enabled").setBoolean(false);
    }

    public static boolean isEnabled() {
        return jetson.getEntry("Enabled").getBoolean(true) == true;
    }

    public static void updateIntakeClosest(){
        try {
            String closest = getClosestIntakeDetection();
            if (closest == ""){
                intakeDetect = null;
            } else {
                intakeDetect = objectMapper.readValue(closest, Detect.class);
            }
        } catch (Exception e) {
            intakeDetect = null;
        }
    }

    public static void updateGripperClosest () {
        try {
            String closest = getClosestGripperDetection();
            if (closest == ""){
                gripperDetect = null;
            } else {
                gripperDetect = objectMapper.readValue(closest, Detect.class);
            }
        } catch (Exception e){
            gripperDetect = null;
        }
    }

}
