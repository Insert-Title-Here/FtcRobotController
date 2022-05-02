package teamcode.common.Movements;

public class Wait extends Movement{
    private long millis;
    public Wait(long millis){
        this.millis = millis;
    }

    public long getMillis() {
        return millis;
    }
}
