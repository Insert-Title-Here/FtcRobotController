package org.firstinspires.ftc.teamcode.Testing.Command.NewBot;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Testing.RobotK;
import org.firstinspires.ftc.teamcode.Vector2D;

import java.io.FileNotFoundException;
@TeleOp
public class CommandTestingTeleOp extends CommandOpMode {

    private RobotK robot;
    private final double NORMAL_LINEAR_MODIFIER = 0.45;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 0.75;
    private double power = 0.055;
    private boolean clampFlag = false;
    private boolean clampState = false;

    //private boolean changePower = true;



    @Override
    public void initialize() {
        try {
            robot = new RobotK(hardwareMap, telemetry);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

    }

    @Override
    public void run() {
        super.run();
        robot.update();

        if (gamepad1.right_bumper) { // replace this with a button for sprint
            robot.drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
        } else {
            robot.drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
        }



        if (gamepad1.b){
            robot.lift.retract(0.5);
        }

        if (gamepad1.a && !robot.lift.extended) {
            robot.lift.extend(1);
        }




        if (gamepad1.dpad_left){
            robot.lift.setPower(0.3);
        }else if (gamepad1.dpad_right){
            robot.lift.setPower(-0.3);
        }else{
            robot.lift.setPower(power);
        }

        if (gamepad1.left_trigger > 0.1) {
            robot.intake.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > 0.1) {
            robot.intake.setPower(gamepad1.right_trigger);
        }


        if(gamepad1.left_bumper && clampFlag){

            robot.intake.clampAndRelease(clampState);
            clampState = !clampState;

            clampFlag = false;

        }

        if(!gamepad1.left_bumper){
            clampFlag = true;
        }

        if (gamepad1.x) {
            robot.intake.setLinkage(0);
        } else if (gamepad1.y) {
            robot.intake.setLinkage(1);
        }

        //will change button later, auto 

        if (gamepad1.back) {
            robot.intake.clampAndRelease(true);
            robot.intake.setLinkage(1);
            sleep(1000);
            robot.lift.extend(1);
        }



/*
        if(gamepad1.dpad_up && changePower){
            power += 0.01;
            changePower = false;
        }else{
            changePower = true;
        }

        if(gamepad1.dpad_down && changePower){
            power -= 0.01;
            changePower = false;
        }else{
            changePower = true;
        }

 */



        //telemetry.addData("Power: ", power);
        //telemetry.addData("Change Power: ", changePower);
        telemetry.addData("Encoder: ", robot.getSpecificEncoderValue(0, false));

        telemetry.update();


    }
}
