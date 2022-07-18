package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    public DcMotor lift;
    public boolean extended;

    public Lift(HardwareMap hardwareMap){
        lift = hardwareMap.get(DcMotor.class, "lift");
        extended = false;

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO: See if we need to set direction
    }

    public void setPower(double power){
        lift.setPower(power);
    }

    public void runToPosition(int tics, double power){
        while(Math.abs(tics - lift.getCurrentPosition()) > 10){
            setPower(power);
        }

        brake();

    }

    public void extend(double power){
        //TODO: tune extension tic value later
        runToPosition(1000, power);
        extended = true;
    }

    public void retract(double power){
        runToPosition(0, power);
        extended = false;
    }

    public void brake(){
        setPower(0);
    }



}
