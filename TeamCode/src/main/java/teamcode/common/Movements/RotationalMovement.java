package teamcode.common.Movements;

public class RotationalMovement extends Movement{
    private double rotation, omega;
    public RotationalMovement(double rotation, double omega){
        this.rotation = rotation;
        this.omega = omega;
    }

    public double getOmega() {
        return omega;
    }

    public double getRotation() {
        return rotation;
    }
}
