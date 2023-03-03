package org.firstinspires.ftc.teamcode.Competition.MTI;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunnerfiles.util.Encoder;

public class Localizer {
    private Encoder leftEncoder, rightEncoder, auxEncoder;

    //TODO: tune these
    private final static double L = 0;  //distance between encoder 1 and 2 in cm (parallel encoder distance)
    private final static double B = 0;  //distance between the midpoint of parallel encoders and perpendicular encoder
    private final static double R = 1.75;  //wheel radius in cm
    private final static double N = 4096;  //encoder ticks in ticks per revolution, AXON REDUX encoder
    private final static double cm_per_tick = 2.0 * Math.PI * R / N;

    private int currentRightPosition = 0, currentLeftPosition = 0, currentAuxPosition = 0;
    private int oldRightPosition = 0, oldLeftPosition = 0, oldAuxPosition = 0;
    private double x = 0, y = 0, heading = 0;


    public Localizer(HardwareMap hardwareMap){
        leftEncoder = hardwareMap.get(Encoder.class, "name");
        rightEncoder = hardwareMap.get(Encoder.class, "name");
        auxEncoder = hardwareMap.get(Encoder.class, "name");


    }


    public void updatePosition(){
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;

        //TODO: Fix negations (either here or set direction)
        currentRightPosition = rightEncoder.getCurrentPosition();
        currentLeftPosition = leftEncoder.getCurrentPosition();
        currentAuxPosition = auxEncoder.getCurrentPosition();

        int deltaN1 = currentLeftPosition - oldLeftPosition;
        int deltaN2 = currentRightPosition - oldRightPosition;
        int deltaN3 = currentAuxPosition - oldAuxPosition;

        double deltaTheta = cm_per_tick * (deltaN1 - deltaN2) / L;
        double deltaX = cm_per_tick * (deltaN1 + deltaN2) / 2.0;
        double deltaY = cm_per_tick * (deltaN3 - (deltaN2 - deltaN1) * B / L);

        double theta = heading + (deltaTheta / 2.0);
        x += deltaX * Math.cos(theta) - deltaY * Math.sin(theta);
        y += deltaX * Math.sin(theta) + deltaY * Math.cos(theta);
        heading += deltaTheta;
    }

    /**
     * Returns x position in terms of centimeters (cm)
     * Used for linear movement?
     * @return
     */
    public double getX(){
        return x;
    }

    /**
     * Returns y position in terms of centimeters (cm)
     * Used for strafing?
     * @return
     */
    public double getY(){
        return y;
    }

    /**
     * Returns heading in terms of radians
     * @return
     */
    public double getHeading(){
        return heading;
    }
}
