package org.firstinspires.ftc.teamcode.Testing.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Vector2D;
import org.firstinspires.ftc.teamcode.roadrunnerfiles.util.Encoder;

@TeleOp
public class OdoTeleopTest extends LinearOpMode {

    MecDriveSimple drive;
    Servo servo;
    private Encoder leftEncoder, rightEncoder, frontEncoder, ll, ll2, rl, rl2, br;

    private final double NORMAL_LINEAR_MODIFIER = 0.5;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.5;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LeftLift2"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RightLift2"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RightLift"));

        ll = new Encoder(hardwareMap.get(DcMotorEx.class, "LeftLift"));
        ll2 = new Encoder(hardwareMap.get(DcMotorEx.class, "LeftLift2"));
        rl = new Encoder(hardwareMap.get(DcMotorEx.class, "RightLift"));
        rl2 = new Encoder(hardwareMap.get(DcMotorEx.class, "RightLift2"));
        br = new Encoder(hardwareMap.get(DcMotorEx.class, "BackRightDrive"));


        // center = RightLift
        // left = LeftLift2
        // right = RightLift2

        drive = new MecDriveSimple(hardwareMap, telemetry);

        //servo = hardwareMap.get(Servo.class, "turret");


        waitForStart();


        while (opModeIsActive()) {

            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = gamepad1.left_stick_y;

            if(Math.abs(leftStickX) > Math.abs(leftStickY)){
                leftStickY = 0;

            }else if(Math.abs(leftStickY) > Math.abs(leftStickX)){
                leftStickX = 0;

            }else{
                leftStickY = 0;
                leftStickX = 0;
            }

            drive.setPower(new Vector2D(leftStickX * NORMAL_LINEAR_MODIFIER, -leftStickY * NORMAL_LINEAR_MODIFIER), -gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);


            telemetry.addData("pod 1 - Left", leftEncoder.getCurrentPosition());
            telemetry.addData("pod 2 - Back/Center", frontEncoder.getCurrentPosition());
            telemetry.addData("pod 3 - Right", rightEncoder.getCurrentPosition());
            telemetry.addData("ll", ll.getCurrentPosition());
            telemetry.addData("ll2", ll2.getCurrentPosition());
            telemetry.addData("rl", rl.getCurrentPosition());
            telemetry.addData("rl2", rl2.getCurrentPosition());
            telemetry.addData("br", br.getCurrentPosition());
            telemetry.update();

            if(gamepad1.a){
                servo.setPosition(0);
            }

            if(gamepad1.b){
                servo.setPosition(1);
            }

        }
    }

}