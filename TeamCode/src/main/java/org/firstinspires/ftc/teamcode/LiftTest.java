package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class LiftTest extends LinearOpMode {
    DcMotor liftMotor;


    @Override
    public void runOpMode() throws InterruptedException {
        liftMotor = hardwareMap.get(DcMotor.class, "motor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Not actually without encoder (just dont use given PID)
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        waitForStart();


        //goToPosition();

        while(opModeIsActive()){
            telemetry.addData("motorPosition", liftMotor.getCurrentPosition());
            telemetry.update();
        }

        liftMotor.setPower(0);





    }

    public void goToTarget(int tics, double power){

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setTargetPosition(tics);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        liftMotor.setPower(power);
        while(liftMotor.isBusy()){
            telemetry.addData("motorPosition", liftMotor.getCurrentPosition());
            telemetry.update();
        }

        liftMotor.setPower(0);
    }







    public void goToPosition(int tics, double power){

        //Include this if the encoder position goes negative when going up
        int motorPosition = liftMotor.getCurrentPosition()/* * -1*/;

        if(motorPosition > tics){
            power *= -1;
        }

        while((Math.abs(motorPosition - tics) > 10)){

            liftMotor.setPower(power);

            motorPosition = liftMotor.getCurrentPosition();

        }

        liftMotor.setPower(0);

    }
}
