package teamcode.common.Movements;

public class TranslationalMovement extends Movement{
    private int dPosition;
    private double velocity, degrees;
    private boolean brake;

    public TranslationalMovement(int dPosition, double velocity, double degrees) {
        this.dPosition = dPosition;
        this.velocity = velocity;
        this.degrees = degrees;
        this.brake = false;
    }
    public TranslationalMovement(int dPosition, double velocity, double degrees, boolean brake){
        this.dPosition = dPosition;
        this.velocity = velocity;
        this.degrees = degrees;
        this.brake = brake;
    }


    public double getVelocity() {
        return velocity;
    }

    public int getdPosition() {
        return dPosition;
    }

    public double getDegrees() {
        return degrees;
    }

    public boolean getBrake(){
        return brake;
    }
}
