package teamcode.Competition;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;

import teamcode.common.AbstractOpMode;
import teamcode.common.Utils;

public class EndgameSystems {
    /**
     * Class for the Carousel and the Capstone mech
     * Electronics Schematic,
     *
     * Capstone Mechanism
     * 1xMotor
     * 1xServo
     *
     * Carousel
     * 1xCRServo
     * 1xEncoder
     */

    private CRServo carousel;
    private DcMotor carouselEncoder;
    private boolean isBlue;


    public void runCarousel(double power) {
        double direction;
        if(isBlue){
            direction = 1;
        }else {
            direction = -1;
        }
        carousel.setPower(power * direction);
    }




    public EndgameSystems(HardwareMap hardwareMap, boolean isBlue){
        carousel = hardwareMap.crservo.get("Carousel");


        carouselEncoder = hardwareMap.dcMotor.get("Winch");


        carouselEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        capstone.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        capstone.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        capstone.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.isBlue = isBlue;

    }




    public void scoreDuck() {
        carouselEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int pose = 40000;
        double direction;
        if(isBlue){
            direction = 1;
        }else {
            direction = -1;
        }
        pose *= direction;
        carouselEncoder.setTargetPosition(pose);

        carouselEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Math.abs(carouselEncoder.getCurrentPosition()) < Math.abs(carouselEncoder.getTargetPosition()) && AbstractOpMode.currentOpMode().opModeIsActive()){
            AbstractOpMode.currentOpMode().telemetry.addData("target", pose);
            AbstractOpMode.currentOpMode().telemetry.addData("current", carouselEncoder.getCurrentPosition());
            AbstractOpMode.currentOpMode().telemetry.update();
            if(carouselEncoder.getCurrentPosition() > carouselEncoder.getTargetPosition() * 0.25){
                if(carouselEncoder.getCurrentPosition() > carouselEncoder.getTargetPosition() * 0.4){
                    carousel.setPower(1 * direction);
                }else{
                    carousel.setPower(0.75 * direction);
                }
            }else{
                carousel.setPower(0.5 * direction);
            }

        }

        carouselEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setPower(0);
    }


    public void scoreDuckAuto() {
        carouselEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int pose = 40000;
        double direction;
        if(isBlue){
            direction = 1;
        }else {
            direction = -1;
        }
        pose *= direction;
        carouselEncoder.setTargetPosition(pose);
        carouselEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(carouselEncoder.getCurrentPosition() < carouselEncoder.getTargetPosition() && AbstractOpMode.currentOpMode().opModeIsActive()){
            carousel.setPower(.2 * direction);
        }
        carousel.setPower(0);
        carouselEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
