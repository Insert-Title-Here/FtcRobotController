package teamcode.common;

import com.qualcomm.robotcore.hardware.configuration.typecontainers.UserConfigurationType;

public class Movement {

    /***
     * CONSTRUCTOR GUIDE FOR THIS CLASS
     * linear movement: int double double
     * Rotational movement: double double
     * Wall or Warehouse Localization or intake driving function: double MovementType
     * Wall localization = WALL_LOCALIZATION
     * Warehouse Localization = WAREHOUSE_LOCALIZATION
     * intake driving = WAREHOUSE_OPERATION
     * Sleep: long
     * flag modification: int boolean
     * to input a brake command, you can call a pause constructor with a parameter of 0
     */


    //translational params
    private int distance;
    private double velocity, radians;

    //rotational params
    private double rotation;
    private double omega;

    //sleep params
    private long millis;

    //boolean flag modifier params
    private int index;
    private boolean val;

    //indicator
    private MovementType movement;

    //please note when using this class, as with MecanumDriveTrain, all angles are in RADIANS and parameters in Degrees

    /**
     * Constructor for translational splices
     * @param dPosition change in position
     * @param velocity velocity
     * @param degrees angle of movement
     */
    public Movement(int dPosition, double velocity, double degrees){
        distance =  dPosition;
        this.velocity = velocity;
        this.radians = Math.toRadians(degrees);
        movement = MovementType.TRANSLATION;
    }

    /**
     * rotational splices
     * @param rotation angle in degrees
     * @param omega angular velocity
     */
    public Movement(double rotation, double omega){
        this.rotation = Math.toRadians(rotation);
        this.omega = omega;
        movement = MovementType.ROTATION;
    }

    /**
     * should be used for wall or warehouse localization
     * @param velocity velocity
     * @param movement movementType, wall or warehouse localization
     */
    public Movement(double velocity, MovementType movement){
        this.movement = movement;
        this.velocity = velocity;
    }

    /**
     * sleep constructor
     * @param millis time to sleep
     */
    public Movement(long millis){
        this.millis = millis;
        movement = MovementType.PAUSE;
    }

    /**
     * generic movement constructor
     * @param movement movement type
     */
    public Movement(MovementType movement){
        this.movement = movement;
    }

    public Movement(int index, boolean val){
        this.index = index;
        this.val = val;
        movement = MovementType.MODIFY_FLAG;
    }



    public enum MovementType{
        TRANSLATION,
        ROTATION,
        PAUSE,
        WALL_LOCALIZATION,
        WAREHOUSE_LOCALIZATION,
        MODIFY_FLAG,
        WAREHOUSE_OPERATION
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

    public int getIndex() {
        return index;
    }

    public boolean getVal(){
        return val;
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

    public void setIndex(int index) {
        this.index = index;
    }

    public void setVal(boolean val) {
        this.val = val;
    }
}
