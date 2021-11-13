package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Disabled
@Autonomous(name="anfnafh")
public class DriveDistance extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor[] motors;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        waitForStart();

        //method called driveDistance(int tics, double power)

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int tics = 500; //arbetrary value, needs to be parametric

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(tics);
        frontRight.setTargetPosition(tics);
        backLeft.setTargetPosition(tics);
        backRight.setTargetPosition(tics);

        while(Math.abs(frontLeft.getCurrentPosition() - frontLeft.getTargetPosition()) > 10){ // repeat this conditional for all 4 wheels and link with &&, if inconsistent results arise link with ||
            frontLeft.setPower(0.5); //arbetrary value, needs to be parametric
            frontRight.setPower(0.5); //also refer to the mecanum diagram if they need to strafe the negatives need to be mapped
            backLeft.setPower(0.5); //Linear + + + +,  Strafe - + + -, Rotational + - + -
            backRight.setPower(0.5);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);



    }
}
