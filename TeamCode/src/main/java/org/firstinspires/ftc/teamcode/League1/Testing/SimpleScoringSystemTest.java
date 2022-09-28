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


    ScoringSystem score;
    Constants constants;
    Robot robot;
    MecDrive drive;
    ColorRangeSensor color;

    @Override
    public void runOpMode() throws InterruptedException {
        constants = new Constants();
        score = new ScoringSystem(hardwareMap,constants, false);
        robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap, robot, false, telemetry);
        color = hardwareMap.get(ColorRangeSensor.class, "color");



        waitForStart();

        while(opModeIsActive()){

            if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            } else {
                drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }


            /*
            if(gamepad1.right_trigger > 0.1){
                score.setPower(gamepad1.right_trigger);

            }else if(gamepad1.left_trigger > 0.1){
                score.setPower(-gamepad1.left_trigger);
            }else{
                score.setPower(0);
            }
*/

            if(gamepad1.dpad_down){
                //score.linkageAutomated(false);
                score.setLinkagePosition(0.05);

            }

            if(gamepad1.dpad_right){
                //score.linkageAutomated(true);
                score.setLinkagePosition(0.42);

            }

            if(gamepad1.dpad_up){
                score.setLinkagePosition(0.95);
            }

            if(gamepad1.a){
                score.grabberOpenAndClose(false);
            }else if(gamepad1.b || color.getDistance(DistanceUnit.CM) < 6.5){
                score.grabberOpenAndClose(true);
            }

            /*
            if(gamepad1.x){
                score.reset();
            }

            if(gamepad1.y){
                score.moveToPosition(550, 0.5);
            }

            if(!score.isBusy()){
                score.setPower(0.04);

            }
*/
            telemetry.addData("rMotor", score.getEncoderPosition(true));
            telemetry.addData("lMotor", score.getEncoderPosition(false));
            telemetry.update();

        }
    }
}
