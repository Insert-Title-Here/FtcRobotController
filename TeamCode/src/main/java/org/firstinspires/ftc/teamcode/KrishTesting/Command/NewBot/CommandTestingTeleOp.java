package org.firstinspires.ftc.teamcode.KrishTesting.Command.NewBot;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.KrishTesting.RobotK;
import org.firstinspires.ftc.teamcode.Vector2D;

import java.io.FileNotFoundException;
@TeleOp
public class CommandTestingTeleOp extends CommandOpMode {

    private RobotK robot;
    private final double NORMAL_LINEAR_MODIFIER = 0.45;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 0.75;
    private double power = 0.06;
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



        if(gamepad1.b){
            robot.lift.retract(0.2);
        }

        if(gamepad1.a && !robot.lift.extended) {
            robot.lift.extend(0.5);
        }




        if(gamepad1.right_trigger > 0.1){
            robot.lift.setPower(0.4);
        }else if(gamepad1.left_trigger > 0.1){
            robot.lift.setPower(-0.4);
        }else{
            robot.lift.setPower(power);
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



        telemetry.addData("Power: ", power);
        //telemetry.addData("Change Power: ", changePower);

        telemetry.update();


    }
}
