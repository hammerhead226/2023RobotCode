package frc.libs.electronics.encoders;

public interface ThreadedEncoder<T> {

    public T getEncoder();

    public void trackOverflows();

    public double getRawPosition();

    public double getContinuousPosition();

    public double getOffsetPosition();


}
