package org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.CommandGroup.TalonsLiftingSequence;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.CommandGroup.TalonsRampScore;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.CommandGroup.TalonsSlideScore;
import org.firstinspires.ftc.teamcode.Vector2D;

import java.io.FileNotFoundException;

@TeleOp
public class TalonsCommandTestingTeleOp extends CommandOpMode {

    private RobotT robot;
    private final double NORMAL_LINEAR_MODIFIER = 0.45;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;


    ElapsedTime timer;
    private boolean timerFlag;

    public enum OpModeType {
        TELEOP, AUTO
    }



    @Override
    public void initialize() {
        try {
            robot = new RobotT(hardwareMap);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        timer = new ElapsedTime();
        timerFlag = true;

    }



    @Override
    public void run() {

        if(timer.seconds() > 30 && timer.seconds() < 60 && timerFlag){
            gamepad1.rumble(500);
            timerFlag = false;

            //Insert LED strip light color change
            robot.randomColor();
        }

        if(timer.seconds() > 60 && timer.seconds() < 90 && !timerFlag){
            gamepad1.rumble(500);
            timerFlag = true;
            telemetry.addData("In here", timerFlag);

            //Insert LED strip light color change
            robot.randomColor();
        }

        if(timer.seconds() > 90 && timerFlag){
            gamepad1.rumble(1000);
            timerFlag = false;

            telemetry.addData("In here2", timerFlag);


            //Insert LED strip light color change
            robot.randomColor();
        }
        super.run();
        robot.update();

        if (gamepad1.right_bumper) { // replace this with a button for sprint
            robot.drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
        } else {
            robot.drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
        }



        if(gamepad1.a) {

            //robot.intake.runIntake = !robot.intake.runIntake;
            //.lift.up();
            //robot.lift.extend(0.5);
            //switchIntake = false;
            schedule(new TalonsLiftingSequence(OpModeType.TELEOP, robot, gamepad1));
        }

        if(gamepad1.b){
            schedule(new TalonsRampScore(OpModeType.TELEOP, robot));

        }

        if(gamepad1.y){
            //schedule(new TalonsSlideScore(OpModeType.TELEOP, robot));
            robot.lift.runToPosition(5000, 0.5);
        }




        if(gamepad1.right_trigger > 0.1){
            robot.lift.setRampPower(gamepad1.right_trigger);
            //robot.lift.setPower(gamepad1.right_trigger);
        }else if(gamepad1.left_trigger > 0.1){
            robot.lift.setRampPower(-gamepad1.left_trigger);
            //robot.lift.setPower(-gamepad1.left_trigger);
        }else{
            robot.lift.setRampPower(0);
        }

        if(robot.intake.runIntake == true){
            robot.intake.setPower(1);
        }else{
            robot.intake.setPower(0);
        }





        if(robot.rgba != null) {
            telemetry.addData("Normalized rgba alpha", robot.rgba.alpha);
            telemetry.addData("Normalized rgba red", robot.rgba.red);
            telemetry.addData("Normalized rgba green", robot.rgba.green);
            telemetry.addData("Normalized rgba blue", robot.rgba.blue);
        }
        telemetry.addData("Distance", robot.color.getDistance(DistanceUnit.INCH));
        telemetry.addData("extended: ", robot.lift.isExtended());
        telemetry.addData("lift encoder: ", robot.lift.getPosition(true));
        telemetry.addData("lift : ", robot.lift.getPosition(false));
        telemetry.addData("run to position (extend): ", Math.abs(Math.abs(robot.lift.liftEncoder.getCurrentPosition()) - 5000));





        telemetry.update();


    }







}
