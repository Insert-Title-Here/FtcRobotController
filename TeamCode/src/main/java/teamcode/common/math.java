package teamcode.common;

public class math {
    public static Vector2D TakeFourWheelsAndConvertAllOfTheir96mmGoBildaWheelMecanumVelocitiesIntoOneBigVectorForTheMethodThatThisIsBeingCalledInForThreeProgrammersWhoDontKnowHowToWriteTheirOwn(Vector2D a, Vector2D b, Vector2D c, Vector2D d){

        Vector2D v = a.add(b.add(c.add(d)));
        return v.multiply(0.25);
    }
}
