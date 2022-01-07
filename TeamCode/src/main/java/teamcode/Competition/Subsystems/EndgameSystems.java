package teamcode.Competition.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.Utils;
import teamcode.common.WestCoastDriveTrain;

public class EndgameSystems {
    /**
     * Class for the Carousel and the Capstone mech
     * Electronics Schematic,
     *
     * Capstone Mechanism
     * 1x crServo
     * 2xServo
     *
     * Carousel
     * 1xCRServo
     * 1xEncoder
     */

    private CRServo carousel, capstoneExtension;
    private Servo xCap, yCap;
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

        if(isBlue) {
            carousel = hardwareMap.crservo.get("CarouselBlue");
            carouselEncoder = hardwareMap.dcMotor.get("BackRightDrive");
        }else{
            //carousel = hardwareMap.crservo.get("CapstoneServo");
            //carouselEncoder = hardwareMap.dcMotor.get("CarouselRedEncoder");
        }
        xCap = hardwareMap.servo.get("xCap");
        yCap = hardwareMap.servo.get("yCap");
        capstoneExtension = hardwareMap.crservo.get("capExtension");

        //carouselEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //carouselEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.isBlue = isBlue;

    }

    public void setXCapPosition(double pos){
        xCap.setPosition(pos);
    }

    public double getXCapPosition() {
        return xCap.getPosition();
    }

    public void setCapstoneExtensionPower(double pow){
        capstoneExtension.setPower(pow);
    }

    public void setYCapPosition(double pos){
        yCap.setPosition(pos);
    }

    public double getYCapPosition() {
        return yCap.getPosition();
    }

    public void zeroCap() {
        xCap.setPosition(0);
        yCap.setPosition(1);
    }

    public void scoreDuck() {
        carouselEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int pose = 20000;
        long currentTime = System.currentTimeMillis();

        carouselEncoder.setTargetPosition(pose);

        carouselEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(Math.abs(carouselEncoder.getCurrentPosition()) < Math.abs(carouselEncoder.getTargetPosition()) && AbstractOpMode.currentOpMode().opModeIsActive()){
            AbstractOpMode.currentOpMode().telemetry.addData("target", pose);
            AbstractOpMode.currentOpMode().telemetry.addData("current", carouselEncoder.getCurrentPosition());
            AbstractOpMode.currentOpMode().telemetry.update();
            carousel.setPower(0.8);
//            if(carouselEncoder.getCurrentPosition() > carouselEncoder.getTargetPosition() * 0.1){
//                if(carouselEncoder.getCurrentPosition() > carouselEncoder.getTargetPosition() * 0.2){
//                    carousel.setPower(1 );
//                }else{
//                    carousel.setPower(0.75 );
//                }
//            }else{
//                carousel.setPower(0.5 );
//            }


        }

        carouselEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setPower(0);
    }


    public void scoreDuckAuto() {
        carouselEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int pose = 40000;
        double direction;
        if(isBlue){
            direction = -1;
        }else {
            direction = 1;
        }
        pose *= direction;
        carouselEncoder.setTargetPosition(pose);
        carouselEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Math.abs(carouselEncoder.getCurrentPosition()) < Math.abs(carouselEncoder.getTargetPosition()) && AbstractOpMode.currentOpMode().opModeIsActive()){
            carousel.setPower(.1 * direction);
        }
        carousel.setPower(0);
        carouselEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
