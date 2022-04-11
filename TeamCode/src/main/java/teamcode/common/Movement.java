package teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.UserConfigurationType;

public class Movement {

    /***
     * CONSTRUCTOR GUIDE FOR THIS CLASS
     * linear movement: int double double
     * arc movement: double int double
     * Rotational movement: double double
     * Wall or Warehouse Localization or intake driving function: double MovementType
     * Wall localization = WALL_LOCALIZATION
     * Warehouse Localization = WAREHOUSE_LOCALIZATION
     * intake driving = WAREHOUSE_OPERATION
     * Sleep: long
     * flag modification: int boolean
     * to input a brake command, you can call a pause constructor with a parameter of 0
     *
     */


    //translational params
    private double distance;
    private double velocity, radians;
    private double plateu;

    //rotational params
    private double rotation;
    private double omega;

    //sleep params
    private long millis;

    //boolean flag modifier params
    private int index;
    private boolean val;

    private double power;

    //indicator
    private MovementType movement;

    private DcMotor.ZeroPowerBehavior behavior;

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

    public Movement(double degrees, double velocity, int dPosition){
        distance = dPosition;
        this.velocity = velocity;
        this.radians = Math.toRadians(degrees);
        movement = MovementType.ARC_MOVEMENT;
    }

    public Movement(double strafe, long time){
        this.power = strafe;
        this.millis = time;
        this.movement = MovementType.STRAFE_TP;
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

    public Movement(double velocity, int max){
        this.velocity = velocity;
        this.distance = max;
        movement = MovementType.WAREHOUSE_LOCALIZATION;
    }

    public Movement(double velocity, double distance, double plateu){
        this.velocity = velocity;
        this.distance = distance;
        this.plateu = plateu;
        movement = MovementType.COAST_MOVEMENT;
    }

    /**
     * sleep constructor
     * @param millis time to sleep
     */
    public Movement(long millis){
        this.millis = millis;
        movement = MovementType.PAUSE;
    }

    public Movement(double velocity, long millis, double power){
        this.velocity = velocity;
        this.millis = millis;
        this.power = power;
        movement = MovementType.COAST_MOVEMENT;
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

    public Movement(double power){
        this.power = power;
        movement = MovementType.MODULATE_INTAKE;
    }

    public Movement(DcMotor.ZeroPowerBehavior behavior){
        this.behavior = behavior;
        movement = MovementType.MODIFY_ZEROPOWER;
    }




    public enum MovementType{
        TRANSLATION,
        ROTATION,
        PAUSE,
        WALL_LOCALIZATION,
        WAREHOUSE_LOCALIZATION,
        MODIFY_FLAG,
        WAREHOUSE_OPERATION,
        ARC_MOVEMENT,
        MODULATE_INTAKE,
        COAST_MOVEMENT,
        MODIFY_ZEROPOWER,
        STRAFE_TP
    }






    public double getRadians() {
        return radians;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getDistance() {
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

    public void setPower(double power) {
        this.power = power;
    }

    public double getPower() {
        return power;
    }

    public void setBehavior(DcMotor.ZeroPowerBehavior behavior) {
        this.behavior = behavior;
    }

    public DcMotor.ZeroPowerBehavior getBehavior() {
        return behavior;
    }

    public double getPlateu() {
        return plateu;
    }

    public void setPlateu(double plateu) {
        this.plateu = plateu;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }
}
