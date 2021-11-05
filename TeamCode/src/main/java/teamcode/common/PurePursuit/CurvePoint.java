package teamcode.common.PurePursuit;


//point wrapper class
public class CurvePoint {
    double x;
    double y;
    double moveSpeed;
    double turnSpeed;
    double followDistance;
    double pointLength; //may be able to add this sometime if we find it to be ne
    double slowDownTurnRads;
    //slows down robot when overshot
    double slowDownTurnAmount;
    boolean isLast;
    long waitMillis; //millis to wait when the

    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnDegrees, double slowDownTurnAmount){
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        //this.pointLength = pointLength;
        this.slowDownTurnRads = slowDownTurnDegrees;
        this.slowDownTurnAmount = slowDownTurnAmount;
    }

    public CurvePoint(CurvePoint thisPoint){
        x = thisPoint.x;
        y = thisPoint.y;
        moveSpeed = thisPoint.moveSpeed;
        turnSpeed = thisPoint.turnSpeed;
        followDistance = thisPoint.followDistance;
        //pointLength = thisPoint.pointLength;
        slowDownTurnRads = thisPoint.slowDownTurnRads;
        slowDownTurnAmount = thisPoint.slowDownTurnAmount;
        isLast = false;
    }




    public String toString(){
        return "(" + this.x + "," + this.y + ")";
    }
}
