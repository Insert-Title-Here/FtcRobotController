package teamcode.common;

import com.qualcomm.robotcore.hardware.configuration.typecontainers.UserConfigurationType;

public class Movement {
    //translational params
    private int distance;
    private double velocity, radians;

    //rotational params
    private double rotation;
    double omega;

    long millis;

    //indicator
    private MovementType movement;

    //please note when using this class, as with MecanumDriveTrain, all angles are in RADIANS and parameters in Degrees

    public Movement(int dPosition, double velocity, double degrees){
        distance =  dPosition;
        this.velocity = velocity;
        this.radians = Math.toRadians(degrees);
        movement = MovementType.TRANSLATION;
    }

    public enum MovementType{
        TRANSLATION, ROTATION, PAUSE, IDLE_DRIVE, SENSOR
    }




    public Movement(double rotation, double omega){
        this.rotation = Math.toRadians(rotation);
        this.omega = omega;
        movement = MovementType.ROTATION;
    }
    public Movement(long millis){
        this.millis = millis;
    }
    public Movement(MovementType movement){
        this.movement = movement;
    }


    public double getRadians() {
        return radians;
    }

    public double getVelocity() {
        return velocity;
    }

    public int getDistance() {
        return distance;
    }

    public double getRotation() {
        return rotation;
    }

    public double getOmega() {
        return omega;
    }

    public MovementType getMovement() {
        return movement;
    }

    public long getMillis() {
        return millis;
    }

    public void setDistance(int distance) {
        this.distance = distance;
    }

    public void setMillis(long millis) {
        this.millis = millis;
    }

    public void setMovement(MovementType movement) {
        this.movement = movement;
    }

    public void setOmega(double omega) {
        this.omega = omega;
    }

    public void setRadians(double radians) {
        this.radians = radians;
    }

    public void setRotation(double rotation) {
        this.rotation = rotation;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }
}
