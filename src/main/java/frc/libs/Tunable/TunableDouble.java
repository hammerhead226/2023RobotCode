package frc.libs.Tunable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableDouble extends TunableElement<Double> {
    
    /**
     * @param key : the key in SmartDashboard (string)
     * @param defaultValue : the default value for the element (double)
     */
    public TunableDouble(String key, double defaultValue) {
        SmartDashboard.putNumber(key, defaultValue);
        this.defaultValue = defaultValue;
    }

    /**
     * Gets the default value for this element, instantiated on initialization.
     * @return the default value (double)
     */
    public Double getDefaultValue() {
        return this.defaultValue;
    }

    /**
     * Gets the current value for this element
     * @return the current value (double)
     */
    public Double getValue() {
        return SmartDashboard.getNumber(key, defaultValue);
    }
}
