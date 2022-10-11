package org.firstinspires.ftc.teamcode.League1.Testing.TeleOpTesting;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Common.Vector2D;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;


//TODO: figure out bulk read

@TeleOp
public class SimpleScoringSystemTest extends LinearOpMode {

    private final double NORMAL_LINEAR_MODIFIER = 0.5;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.5;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;

    DcMotorEx rLift, lLift;


    ScoringSystem2 score;
    Constants constants;
    Robot robot;
    MecDrive drive;
    ColorRangeSensor distance, color;

    boolean autoLinkageFlag = true;
    boolean grabFlag = true;

    @Override
    public void runOpMode() throws InterruptedException {
        constants = new Constants();
        score = new ScoringSystem2(hardwareMap, constants);
        robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap, robot, false, telemetry);
        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        color = hardwareMap.get(ColorRangeSensor.class, "color");

        rLift = hardwareMap.get(DcMotorEx.class, "RightLift");
        lLift = hardwareMap.get(DcMotorEx.class, "LeftLift");

        rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        color.setGain(300);
        distance.setGain(300);

        score.setLinkagePosition(0.03);
        score.setGrabberPosition(0.75);

        /*score.lLift = hardwareMap.get(DcMotor.class, "LeftLift");
        score.rLift = hardwareMap.get(DcMotor.class, "RightLift");*/

        //score.resetAndRunWithoutEncoders();

        //score.correctMotors();





        waitForStart();

        while(opModeIsActive()){



            if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            } else {
                drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }





            if(gamepad1.right_trigger > 0.1){
                setPower(gamepad1.right_trigger);

            }else if(gamepad1.left_trigger > 0.1){
                setPower(-gamepad1.left_trigger);
            }else{
                setPower(0);
            }


            if(gamepad1.b){
                score.moveToPosition(200, 0.8);
            }

            if(gamepad1.a){
                score.moveToPosition(0, 0.5);
            }




            if(gamepad1.left_bumper){
                //score.linkageAutomated(false);
                score.setLinkagePosition(0.7);
            }

            if(gamepad1.right_bumper){
                score.setLinkagePosition(1);
            }

            if(gamepad1.start){
                score.setLinkagePosition(0.2);
                sleep(200);
                score.setLinkagePosition(0.03);
                autoLinkageFlag = true;
            }



            if(gamepad1.dpad_right){
                score.setGrabberPosition(constants.openAuto);
                grabFlag = true;
            }else if((gamepad1.dpad_left ||  distance.getDistance(DistanceUnit.CM) < 6.5) && grabFlag) {
                score.setGrabberPosition(constants.grabbing);
                grabFlag = false;


            }


            if((distance.getNormalizedColors().red > 0.7 || distance.getNormalizedColors().blue > 0.7) && autoLinkageFlag){
                score.setLinkagePosition(0.7);
                autoLinkageFlag = false;
            }




            telemetry.addData("rMotor", score.getRightEncoderPos());
            telemetry.addData("lMotor", score.getLeftEncoderPos());
            telemetry.addData("distance: ", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("distanceRed", distance.getNormalizedColors().red);
            telemetry.addData("distanceBlue", distance.getNormalizedColors().blue);
            telemetry.addData("autoLinkageFlag", autoLinkageFlag);
            telemetry.addData("grabbingFlag", grabFlag);
            telemetry.addData("colorRed: ", color.getNormalizedColors().red);
            telemetry.addData("colorBlue: ", color.getNormalizedColors().blue);
            telemetry.update();

        }

        drive.setPower(0, 0, 0, 0);
        score.setLinkagePosition(0.5);
        sleep(500);
        score.setLinkagePosition(0.05);
        score.moveToPosition(0, 0.5);
        score.setGrabberPosition(constants.openAuto);

    }

    public void setPower(double power){
        rLift.setPower(power);
        lLift.setPower(-power);
    }





}
