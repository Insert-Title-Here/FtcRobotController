package teamcode.common;

public class Constants {

    //Hardware device names
    public static final String LEFT_VERTICAL_ODOMETER_NAME = "FrontLeftDrive";
    public static final String RIGHT_VERTICAL_ODOMETER_NAME = "Intake";
    public static final String HORIZONTAL_ODOMETER_NAME = "BackLeftDrive";
    public static final String LINEAR_SLIDE_ENCODER_NAME = "FrontLeftDrive";

    //other relevant info
    public static final String SAVE_FILE_PATH = "";
    public static final double INITIAL_VELOCITY = 110.0 * Math.PI;


    //Physics Constants
    public static final double GRAVITY_IN_SEC = 386.09;

    //Game Constants
    public static final double TOP_POSITION = 17000; // -14000 TANK   -21000
    public static final double MEDIUM_POSITION = 8000 ; //store these values as ticks, and just go with that, nothing fancy (from 145000 to 6k, because of new gear)
    public static final double BOTTOM_POSITION = 3500;
    public static final double CAP_POSITION = 16000;



}
