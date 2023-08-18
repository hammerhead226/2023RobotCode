package frc.libs.Tunable;

public abstract class TunableElement<T> {
    
    String key;
    T defaultValue;

    public abstract T getDefaultValue();

    public abstract T getValue();

    public String getKey() { return this.key; }
    
}
