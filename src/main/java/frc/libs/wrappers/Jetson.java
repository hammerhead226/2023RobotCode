package frc.libs.wrappers;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Jetson {
    private static NetworkTable jetson = NetworkTableInstance.getDefault().getTable("Jetson");
    private static ObjectMapper objectMapper = new ObjectMapper();


    public static double getNetworkFPS() {
        return jetson.getEntry("Network FPS").getDouble(0.0);
    }

    public static double getPiplineFPS() {
        return jetson.getEntry("Pipeline FPS").getDouble(0.0);
    }

    public static double getLatency() {
        return jetson.getEntry("Latency").getDouble(0.0);
    }

    public static String getClosestDetection() {
        return jetson.getEntry("Closest Detection").getString("");
    }
}
