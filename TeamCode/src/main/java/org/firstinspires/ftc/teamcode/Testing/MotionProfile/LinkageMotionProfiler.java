package org.firstinspires.ftc.teamcode.Testing.MotionProfile;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class LinkageMotionProfiler {
    ServoImplEx rLinkage, lLinkage;

    public LinkageMotionProfiler(HardwareMap hardwareMap){
        lLinkage = hardwareMap.get(ServoImplEx.class, "LeftLinkage");
        rLinkage = hardwareMap.get(ServoImplEx.class, "RightLinkage");

        lLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));

    }

    public void triangularMotionProfile(double targetPosition, int millis){
        double distance = 355 * Math.abs(targetPosition - rLinkage.getPosition());

        //Average Velocity in degrees per millisecond
        double avgVel = distance / millis;

        double maxVel = 2 * avgVel;

        double slope = maxVel / (millis / 2);



        while(rLinkage.getPosition() != targetPosition){

        }

    }



}
