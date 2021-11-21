package org.firstinspires.ftc.teamcode.TankDriveCode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Carousel Auto (red)", group = "Linear Opmode")

public class CarouselAutoRed extends LinearOpMode {
    DcMotor carousel;
    Servo grabber;


    @Override
    public void runOpMode(){
        DriveTrain drive = new DriveTrain(hardwareMap);

        carousel = hardwareMap.get(DcMotor.class, "Carousel");
        carousel.setDirection(DcMotor.Direction.REVERSE);

        grabber = hardwareMap.get(Servo.class, "Grabber");
        grabber.setPosition(0);

        waitForStart();


        //First turn to carousel
        drive.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.lf.setTargetPosition(200);
        drive.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.lf.setPower(0.5);

        while(drive.lf.isBusy()){

        }

        drive.brake();

        sleep(1000);


        //Carousel spin
        spinCarousel(-4000);


        /*sleep(1000);

        drive.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.lf.setTargetPosition(200);
        drive.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.lf.setPower(0.5);

        while(Math.abs(drive.lf.getCurrentPosition() - drive.lf.getTargetPosition()) > 20){

        }

        drive.brake();
        */

        sleep(1000);

        //drive.goToPosition(300, false);

        //sleep(1000);

        //Backing up away from carousel
        //double startTime = linearOpMode.time;
        drive.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.lf.setTargetPosition(-200);
        drive.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.lf.setPower(0.5);

        while(drive.lf.isBusy()){

        }

        drive.brake();

        sleep(1000);

        //Turning towards parking
        drive.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rf.setTargetPosition(250);
        drive.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.rf.setPower(0.5);



        while(drive.rf.isBusy()){

        }

        drive.brake();

        sleep(1000);


        //Driving into park
        drive.goToPosition(630, false, 0.3);
        sleep(1000);









    }

    public void spinCarousel(int tics) {



        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setTargetPosition(tics);


        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double power;

        carousel.setPower(0.2);

        while(carousel.isBusy()){

        }

        /*while (carousel.isBusy()) {
            power = 2 * (1- (Math.abs(carousel.getCurrentPosition() - carousel.getTargetPosition()) / 4000.0));
            if (power < 0.5) {
                carousel.setPower(0.5);
            } else if (power > 1){
                carousel.setPower(1);
            } else {
                carousel.setPower(power);
            }
        }

         */

        carousel.setPower(0);

        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

}
