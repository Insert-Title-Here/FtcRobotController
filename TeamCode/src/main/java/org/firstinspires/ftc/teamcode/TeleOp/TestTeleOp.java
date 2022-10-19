package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.MaintainLiftPosition;
import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

@TeleOp
public class TestTeleOp extends LinearOpMode {

    MecanumDrive drive;

    //TODO: Move scoring system stuff to its own class
    static DcMotor lift, rightLift;
    static Servo claw, fourbar;


    private final double NORMAL_LINEAR_MODIFIER = 0.6;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.6;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;


    int origLiftPos, origRightLiftPos;
    int liftPos;
    MaintainLiftPosition maintainLiftPos = new MaintainLiftPosition();

    static boolean liftIsStill = false;
    boolean clawIsClosed = true;
    static int targetPos, rightTargetPos;
    static double origFourbarPos;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, telemetry);
        lift = hardwareMap.get(DcMotor.class, "lift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        claw = hardwareMap.get(Servo.class, "claw");
        fourbar = hardwareMap.get(Servo.class, "fourbar");
        origLiftPos = lift.getCurrentPosition();
        origRightLiftPos = rightLift.getCurrentPosition();
        origFourbarPos = fourbar.getPosition();



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        while(opModeIsActive()){

            if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(gamepad1.right_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.left_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            }
            else {
                drive.setPower(new Vector2D(gamepad1.right_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.left_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }

            if(gamepad1.dpad_left){
                liftIsStill = false;
                while(lift.getCurrentPosition()<origLiftPos+1500&&opModeIsActive()){
                    lift.setPower(1);
                    rightLift.setPower(-1);
                }

                while(lift.getCurrentPosition()>origLiftPos+2700&&opModeIsActive()){
                    lift.setPower(-1);
                    rightLift.setPower(1);
                }
                maintainLiftPos(origLiftPos+2140, origRightLiftPos-2140);
                openFourbar();
                sleep(200);

            } else if(gamepad1.dpad_up){
                liftIsStill = false;
                while(lift.getCurrentPosition()<origLiftPos+3300&&opModeIsActive()){
                    lift.setPower(1);
                    rightLift.setPower(-1);
                    telemetry.addData("liftPos", lift.getCurrentPosition());
                    telemetry.addData("targetLiftPos", origLiftPos+3300);

                }

                while(lift.getCurrentPosition()>origLiftPos+4200&&opModeIsActive()){
                    lift.setPower(-1);
                    rightLift.setPower(1);
                }
                maintainLiftPos(origLiftPos+3800, origRightLiftPos-3800);
                openFourbar();

            } else if(gamepad1.dpad_down){
                liftIsStill = false;
                while(lift.getCurrentPosition()<origLiftPos-600&&opModeIsActive()) {
                    lift.setPower(1);
                    rightLift.setPower(-1);
                }

                while(lift.getCurrentPosition()>origLiftPos+550&&opModeIsActive()) {
                    lift.setPower(-1);
                    rightLift.setPower(1);
                }
                maintainLiftPos(origLiftPos, origRightLiftPos);
                closeFourbar();
            }

            if (gamepad1.left_trigger > 0.1) {
                liftIsStill = false;
                lift.setPower(gamepad1.left_trigger*0.8);
                rightLift.setPower(-gamepad1.left_trigger*0.8);
                liftPos = lift.getCurrentPosition();
                telemetry.addData("liftPos", lift.getCurrentPosition());
                telemetry.update();

            } else if (gamepad1.right_trigger > 0.1 && lift.getCurrentPosition()>origLiftPos+10) {
                liftIsStill = false;
                lift.setPower(-gamepad1.right_trigger*0.6);
                rightLift.setPower(gamepad1.right_trigger*0.6);

                //liftPos = lift.getCurrentPosition();
                //liftIsUp = true;
                telemetry.addData("liftPos", lift.getCurrentPosition());
                telemetry.update();

            } else {
                /*if(liftIsUp){
                    while(opModeIsActive()&&lift.getCurrentPosition()<(liftPos-50)) {
                        lift.setPower(0.1);
                        telemetry.addData("Looping", "Adjusting lift position");
                    }
                }*/
                lift.setPower(0);
                rightLift.setPower(0);
            }

            if(gamepad1.a){
                liftIsStill=!liftIsStill;
                if(liftIsStill){
                    maintainLiftPos(lift.getCurrentPosition(), rightLift.getCurrentPosition());
                }
            }

            if(gamepad1.x){
                origLiftPos=lift.getCurrentPosition();
            }

            if(lift.getCurrentPosition()==origLiftPos){
                liftIsStill=false;
            }

            if(gamepad1.left_bumper){
                if(clawIsClosed){
                    claw.setPosition(0);
                    sleep(200);
                } else {
                    claw.setPosition(0.6);
                    sleep(200);
                }
                clawIsClosed = !clawIsClosed;
            }




            telemetry.addData("flPos", drive.getFLPosition());
            telemetry.addData("frPos", drive.getFRPosition());
            telemetry.addData("blPos", drive.getBLPosition());
            telemetry.addData("brPos", drive.getBRPosition());
            telemetry.addData("liftPos", lift.getCurrentPosition());
            telemetry.addData("rightLiftPos", rightLift.getCurrentPosition());
            telemetry.addData("origLiftPos", origLiftPos);
            telemetry.addData("origRightLiftPos", origRightLiftPos);
            telemetry.addData("fourbarPos", fourbar.getPosition());
            telemetry.addData("origFourbarPos", origFourbarPos);



            telemetry.update();

        }

        drive.setPower(0, 0, 0, 0);
    }

    public void maintainLiftPos(int target, int reverseTarget){
        targetPos =  target;
        rightTargetPos = reverseTarget;
        liftIsStill = true;
        Thread autoAdjustLift= new Thread(maintainLiftPos);
        autoAdjustLift.start();
    }

    public static DcMotor getLift(){
        return lift;
    }

    public static DcMotor getRightLift(){
        return rightLift;
    }

    public static int getTargetPosition(){
        return targetPos;
    }

    public static int getRightTargetPosition(){
        return rightTargetPos;
    }

    public static boolean liftIsStill(){
        return (liftIsStill);
    }

    public static void openFourbar() {
        fourbar.setPosition(origFourbarPos);
    }

    public static void closeFourbar() {
        fourbar.setPosition(origFourbarPos+0.5);
    }

}