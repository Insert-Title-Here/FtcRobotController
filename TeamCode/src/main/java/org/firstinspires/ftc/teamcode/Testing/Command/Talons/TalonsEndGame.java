package org.firstinspires.ftc.teamcode.Testing.Command.Talons;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TalonsEndGame {

    private CRServo carouselRed, carouselBlue, xCap;
    private Servo yCap;
    private DcMotor capping;

    public TalonsEndGame(HardwareMap hardwareMap){
        carouselBlue = hardwareMap.get(CRServo.class, "carouselBlue");
        carouselRed = hardwareMap.get(CRServo.class, "carouselRed");
        xCap = hardwareMap.get(CRServo.class, "xCap");
        yCap = hardwareMap.get(Servo.class, "yCap");
        capping = hardwareMap.get(DcMotor.class, "liftEncoder");

    }


    public void runCarouselAuto(){

        carouselBlue.setPower(0.2);
        carouselRed.setPower(0.2);

    }

    public void brakeCarousel(){
        carouselRed.setPower(0);
        carouselBlue.setPower(0);
    }

    public void setxCapPower(double power){
        xCap.setPower(power);
    }

    public void jankUpY(){
        yCap.setPosition(yCap.getPosition() + 0.001);
    }

    public void jankDownY(){
        yCap.setPosition(yCap.getPosition() - 0.001);
    }

    public void setYPosition(double position){
        yCap.setPosition(position);
    }

    public double getYPosition(){
        return yCap.getPosition();
    }

    public void setCappingPower(double power){
        capping.setPower(power);
    }

    public double map(double val, double in_min, double in_max, double out_min, double out_max){
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    }
}
