package org.firstinspires.ftc.teamcode.NewMecanumCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {
    DcMotor carousel;

    public Carousel(HardwareMap hardwareMap) {
        carousel = hardwareMap.dcMotor.get("Carousel");

        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void spin(double power) {
        carousel.setPower(power);
    }

    public void stop() {
        carousel.setPower(0);
    }
}