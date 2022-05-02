package teamcode.common.Movements;

public class WarehouseNormalization extends Movement{
    private double velocity;
    private int max;
    private boolean isCoast;
    public WarehouseNormalization(double velocity, int max, boolean isCoast){
        this.max = max;
        this.velocity = velocity;
        this.isCoast = isCoast;
    }

    public double getVelocity() {
        return velocity;
    }

    public int getMax() {
        return max;
    }

    public boolean getIsCoast(){
        return isCoast;
    }
}
