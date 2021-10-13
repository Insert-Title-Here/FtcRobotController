package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {

    DcMotor carousel;

    public DriveTrain(HardwareMap hardwareMap) {
        carousel  = hardwareMap.get(DcMotor.class, "Carousel");
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void CarouselStop() {
        carousel.setPower(0);
    }

    public void SpinCarousel(int tics, double power) {
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        carousel.setTargetPosition(tics);

        carousel.setPower(power);

        while (carousel.isBusy()) {

        }

        carousel.setPower(0);

        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void SpinCarouselTime(int millis) {
        carousel.setPower(1);
        Thread.sleep(millis);
        carousel.setPower(0);
    }
}
