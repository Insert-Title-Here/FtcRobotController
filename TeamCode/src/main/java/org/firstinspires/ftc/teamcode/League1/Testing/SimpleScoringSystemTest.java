package org.firstinspires.ftc.teamcode.League1.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Common.Vector2D;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;


//TODO: figure out bulk read

@TeleOp
public class SimpleScoringSystemTest extends LinearOpMode {

    private final double NORMAL_LINEAR_MODIFIER = 0.5;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.5;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;

    private boolean grabFlag = true;


    ScoringSystem score;
    Constants constants;
    Robot robot;
    MecDrive drive;
    ColorRangeSensor distance, color;

    @Override
    public void runOpMode() throws InterruptedException {
        constants = new Constants();
        score = new ScoringSystem(hardwareMap,constants, false);
        robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap, robot, false, telemetry);
        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        color = hardwareMap.get(ColorRangeSensor.class, "color");

        color.setGain(300);


        waitForStart();

        while(opModeIsActive()){



            if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            } else {
                drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }





            if(gamepad1.right_trigger > 0.1){
                score.setPower(gamepad1.right_trigger);

            }else if(gamepad1.left_trigger > 0.1){
                score.setPower(-gamepad1.left_trigger);
            }else{
                score.setPower(0);
            }

            if(gamepad1.b){
                score.moveToPosition(300, 0.5);
            }

            if(gamepad1.a){
                score.moveToPosition(0, 0.5);
            }


            if(gamepad1.left_bumper){
                //score.linkageAutomated(false);
                score.setLinkagePosition(0.5);
            }

            if(gamepad1.right_bumper){
                score.setLinkagePosition(0.95);
            }

            if(gamepad1.start){
                score.setLinkagePosition(0.05);
            }


            if(gamepad1.x && grabFlag){
                if(score.getGrabberPosition() == constants.openAuto){
                    score.setGrabberPosition(constants.grabbing);

                }else{
                    score.setGrabberPosition(constants.openAuto);
                }


                grabFlag = false;
            }else{
                grabFlag = true;
            }


            telemetry.addData("rMotor", score.getEncoderPosition(true));
            telemetry.addData("lMotor", score.getEncoderPosition(false));
            telemetry.addData("distance: ", distance.getDistance(DistanceUnit.CM));
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
}
