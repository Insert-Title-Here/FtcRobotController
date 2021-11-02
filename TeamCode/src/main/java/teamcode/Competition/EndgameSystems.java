package teamcode.Competition;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
        carousel.setPower(power);
    }




    public EndgameSystems(HardwareMap hardwareMap, boolean isBlue){
        carousel = hardwareMap.crservo.get("Carousel");

        capstoneMechanism = hardwareMap.servo.get("CapstoneServo");

        capstone = hardwareMap.dcMotor.get("Capstone");
        carouselEncoder = hardwareMap.dcMotor.get("Winch");

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
        int pose;
        if(isBlue){
            pose = (40000);
        }else {
            pose = (int)(40000 * (4.0/3.0));
        }
        carouselEncoder.setTargetPosition(pose);
        carouselEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(carouselEncoder.getCurrentPosition() < carouselEncoder.getTargetPosition() && AbstractOpMode.currentOpMode().opModeIsActive()){
            if(carouselEncoder.getCurrentPosition() > carouselEncoder.getTargetPosition() * 0.25){
                if(carouselEncoder.getCurrentPosition() > carouselEncoder.getTargetPosition() * 0.4){
                    carousel.setPower(1);
                }else{
                    carousel.setPower(0.75);
                }
            }else{
                carousel.setPower(0.5);
            }

        }
        carousel.setPower(0);
    }

    public void setCapstonePower(double power) {
        capstone.setPower(-power);
    }


}
