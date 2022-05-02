package teamcode.common.Movements;

public class ArcMovement extends Movement{

    private int distance;
    private double velocity, radians;

    public ArcMovement(int distance, double velocity, double degrees){
        this.distance = distance;
        this.velocity = velocity;
        this.radians = Math.toRadians(degrees);
    }

    public double getVelocity() {
        return velocity;
    }

    public double getRadians() {
        return radians;
    }

    public int getDistance() {
        return distance;
    }
}
