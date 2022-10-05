package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScoringSystem {
    private DcMotor liftMotor;
    private Servo claw;


    public ScoringSystem(HardwareMap hardwareMap) {
        /* the below is init*/
        claw = hardwareMap.get(Servo.class, "claw");
        liftMotor = hardwareMap.get(DcMotor.class, "motor");

        // reset encoder's tics for liftMotor (leave commented unless you need to reset the encoder for
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Not actually without encoder (just doesn't use given PID)
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // when the power is zero, it'll resist movement/change
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    // goes to given tics at given power -> max tics for arm raising = 4906 -> go for 4800
    public void goToPosition(int tics, double power) {

        int motorPosition = liftMotor.getCurrentPosition();

        if (motorPosition > tics) {
            power *= -1;
        }

        while (Math.abs((Math.abs(tics)-motorPosition)) > 10) {

            liftMotor.setPower(power);
            motorPosition = liftMotor.getCurrentPosition();
            //TODO: use elaspeTime or smth to set power to zero if tics pretty high and power continually being used

        }


        liftMotor.setPower(0);

    }

    public void setPower(double power){
        liftMotor.setPower(power);
    }

    public void setClawPosition(double position) {
        claw.setPosition(position);
    }

    public int getEncoderPosition() {
        return liftMotor.getCurrentPosition();
    }

    public double getClawPosition() {
        return claw.getPosition();
    }
    public void resetLiftEncoder(){
         liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}




