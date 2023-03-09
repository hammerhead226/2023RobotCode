package frc.libs.wrappers;

import com.fasterxml.jackson.databind.PropertyNamingStrategies.UpperCamelCaseStrategy;
import com.fasterxml.jackson.databind.annotation.JsonNaming;

@JsonNaming(UpperCamelCaseStrategy.class)
public class Detect {
    public String classID;
    public String className;
    public String instanceID;
    public double area;
    public double bottom;
    public double centerX;
    public double centerY;
    public double confidence;
    public double width;
    public double height;
    public double left;
    public double right;
    public double top;
    public double timestamp;
    public double targetX;
    public double targetY;
    public double targetDistance;
    public double areaPercent;
}
