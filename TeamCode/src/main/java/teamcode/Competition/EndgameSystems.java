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
    private DcMotor capstone, carouselEncoder;
    private Servo capstoneMechanism;
    private boolean isBlue;

    public void extend(){
        capstoneMechanism.setPosition(1);
    }

    public void scoreCapstone(){
        capstoneMechanism.setPosition(0.5);
    }

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

        capstoneMechanism = hardwareMap.servo.get("CapstoneServo");

        if(isBlue){
            carousel = hardwareMap.crservo.get("Carousel");
            carouselEncoder = hardwareMap.dcMotor.get("Winch");
        }else{
            carousel = hardwareMap.crservo.get("CapstoneServo");
            carouselEncoder = hardwareMap.dcMotor.get("Capstpne");
        }


        carouselEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        capstone.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        capstone.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        capstone.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.isBlue = isBlue;

    }
    public void extendCapstoneMech(){
        goToPosition(-210);
        Utils.sleep(200);
        extend();
        Utils.sleep(200);
        goToPosition(-10);
    }

    public void raiseCapstone(){
       goToPosition(-550);
    }

    public void goToPosition(int position){
        capstone.setTargetPosition(position);
        capstone.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Math.abs(capstone.getCurrentPosition() - capstone.getTargetPosition()) > 10){
            capstone.setPower(-0.8);
            AbstractOpMode.currentOpMode().telemetry.addData("pose", capstone.getCurrentPosition());
            AbstractOpMode.currentOpMode().telemetry.update();
        }
        capstone.setPower(0);
        capstone.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void setCapstonePower(double power) {
        capstone.setPower(-power);
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
