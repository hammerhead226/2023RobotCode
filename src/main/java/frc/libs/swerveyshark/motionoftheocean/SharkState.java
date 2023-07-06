package frc.libs.swerveyshark.motionoftheocean;

public class SharkState {

    private double time;

    private double x;

    private double y;

    private double heading;

    private double linearVelocity;

    private double angularVelocity;

    private double directionalMotion;

    private String event;

    public SharkState(double time, double x, double y, double heading, double linearVelocity, double angularVelocity, double directionalMotion, String event) {
        this.time = time;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.linearVelocity = linearVelocity;
        this.angularVelocity = angularVelocity;
        this.directionalMotion = directionalMotion;
        this.event = event;
    }

    public double getTime() {
        return time;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public double getLinearVelocity() {
        return linearVelocity;
    }

    public double getAngularVelocity() {
        return angularVelocity;
    }

    public double getDirectionalMotion() {
        return directionalMotion;
    }

    public String getEvent() {
        return event;
    }

    public boolean hasEvent() {
        return event.length() > 0;
    }


    public void setTime(double time) {
        this.time = time;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public void setLinearVelocity(double linearVelocity) {
        this.linearVelocity = linearVelocity;
    }

    public void setAngularVelocity(double angularVelocity) {
        this.angularVelocity = angularVelocity;
    }

    public double[] getAsArray() {
        return new double[] {x, y, heading, linearVelocity, angularVelocity, directionalMotion};
    }

    public String toString() {
        return "SharkState{" +
                "time=" + time +
                ", x=" + x +
                ", y=" + y +
                ", heading=" + heading +
                ", linearVelocity=" + linearVelocity +
                ", angularVelocity=" + angularVelocity +
                "}";
    }


}
