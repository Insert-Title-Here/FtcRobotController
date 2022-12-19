package org.firstinspires.ftc.teamcode.League3.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.League3.Common.ScoringSystem;

/* @TeleOp for driver controlled, @Autonomous for autonomous period */
//@Autonomous
public class LiftTest extends LinearOpMode {
    //DcMotor liftMotor;
    ScoringSystem score;
    /*
    every op mode of type linearOpMode must have:
    @Override
    public void runOpMode() {}

     */

    @Override
    public void runOpMode() throws InterruptedException {
        score = new ScoringSystem(hardwareMap, telemetry);
        /* the below is init*/
        /*
        liftMotor = hardwareMap.get(DcMotor.class, "motor");

        // reset encoder's tics for liftMotor
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Not actually without encoder (just doesn't use given PID)
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */

        // https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Creating-and-Running-an-Op-Mode-(Android-Studio)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // when the power is zero, it'll resist movement/change
        //liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //wait for the game to start (driver presses play)
        waitForStart();

        /*
        goToPosition(600, 0.5);
        sleep(1000);
        goToPosition(0, 0.25);
        sleep(500);
        */
        // low cone, 1700 tics  ->>>>> 13, 23, 33 inch elevations
        score.goToPosition(1700, 0.5);
        sleep(5000);
        score.goToPosition(0, 0.25);
        sleep(500);
        // runs until the end of the match (driver presses STOP)
        while(opModeIsActive()){
            telemetry.addData("motorPosition", score.getEncoderPosition());
            telemetry.update();
            // generally

        }

        //liftMotor.setPower(0);
        score.setPower(0);




    }
    /*
    public void goToTarget(int tics, double power){
        // max tics 4906 -> go for 4800

        // make sure to reset motors by hand first!!!
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
    */





    /*
    public void goToPosition(int tics, double power){

        //Include this if the encoder position goes negative when going up
        int motorPosition = liftMotor.getCurrentPosition();

        if(motorPosition > tics){
            power *= -1;
        }

        while((Math.abs(motorPosition - tics) > 10)){

            liftMotor.setPower(power);

            motorPosition = liftMotor.getCurrentPosition();

        }

        liftMotor.setPower(0);

    }
    */
}


