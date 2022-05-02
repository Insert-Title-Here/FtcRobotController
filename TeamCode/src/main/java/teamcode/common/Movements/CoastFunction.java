package teamcode.common.Movements;

public class CoastFunction extends Movement{

    private double velocity, distance, plateu;
    public CoastFunction(double velocity, double distance, double plateu){
        this.velocity = velocity;
        this.distance = distance;
        this.plateu = plateu;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getPlateu() {
        return plateu;
    }

    public double getDistance() {
        return distance;
    }
}
