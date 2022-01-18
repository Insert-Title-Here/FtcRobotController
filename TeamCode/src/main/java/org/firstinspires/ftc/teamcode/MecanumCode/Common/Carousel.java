package org.firstinspires.ftc.teamcode.MecanumCode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {

    public enum CarouselMode {
        AUTO,
        TELEOP
    }

    DcMotor carousel;

    public Carousel(HardwareMap hardwareMap) {
        carousel = hardwareMap.get(DcMotor.class, "Carousel");
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setDirection(DcMotor.Direction.FORWARD);

    }

    public void spinCarousel(int tics, LinearOpMode currentOpMode, CarouselMode mode) {

        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        carousel.setTargetPosition(tics);

        double power;

        if(mode == CarouselMode.AUTO) {
            setPower(0.3);
            while (carousel.isBusy() && currentOpMode.opModeIsActive()) {

            }
        } else {
            carousel.setPower(0.7);
            while (carousel.isBusy() && currentOpMode.opModeIsActive()) {
                power = 2 * (1- (Math.abs(carousel.getCurrentPosition() - carousel.getTargetPosition()) / 4000.0));
                if (power < 0.7) {
                    carousel.setPower(0.7);
                } else if (power > 1){
                    carousel.setPower(1);
                } else {
                    carousel.setPower(power);
                }
            }
        }

        carousel.setPower(0);

        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void setPower(double power) {
        carousel.setPower(power);

    }
}
