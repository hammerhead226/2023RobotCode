package frc.libs.Tunable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableBoolean extends TunableElement<Boolean> {
    
    /**
     * @param key : the key in SmartDashboard (string)
     * @param defaultValue : the default value for the element (boolean)
     */
    public TunableBoolean(String key, Boolean defaultValue) {
        this.key = key;
        // System.out.println(key);
        SmartDashboard.putBoolean(this.key, defaultValue);
        this.defaultValue = defaultValue;
    }

    /**
     * Gets the default value for this element, instantiated on initialization.
     * @return the default value (double)
     */
    public Boolean getDefaultValue() {
        return this.defaultValue;
    }

    /**
     * Gets the current value for this element
     * @return the current value (double)
     */
    public Boolean getValue() {
        // System.out.println(this.key);
        return SmartDashboard.getBoolean(this.key, defaultValue);
    }
}
