package teamcode.common.Movements;

public class WallNormalization extends Movement{

    private double velocity, radians;

    public WallNormalization(double velocity, double radians){
        this.velocity = velocity;
        this.radians = radians;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getRadians() {
        return radians;
    }
}
