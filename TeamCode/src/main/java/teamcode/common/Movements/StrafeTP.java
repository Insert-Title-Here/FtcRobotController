package teamcode.common.Movements;

public class StrafeTP extends Movement{
    private long time;
    private double power;
    public StrafeTP(long time, double power){
        this.time = time;
        this.power = power;
    }

    public double getPower() {
        return power;
    }

    public long getTime() {
        return time;
    }
}
