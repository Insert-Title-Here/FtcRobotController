package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.FileNotFoundException;

@TeleOp(name="Summer Testing TeleOp")
public class SummerTeleOp extends LinearOpMode {

    Thread driveThread;
    Thread intakeThread;
    MecanumDriveTrain drive;
    Intake intake;
    ColorSensor color;



    boolean auto = false;
    boolean changePower = true;
    private double power = 1;
    private final double NORMAL_LINEAR_MODIFIER = 0.45;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 0.75;

    @Override
    public void runOpMode() throws InterruptedException{

        color = hardwareMap.get(ColorSensor.class, "color");


        try {
            drive = new MecanumDriveTrain(hardwareMap);
            intake = new Intake(hardwareMap);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }



        driveThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    driveUpdate();
                }
            }
        };

        intakeThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    intakeUpdate();
                }
            }
        };

        color.enableLed(true);


        waitForStart();

        driveThread.start();
        intakeThread.start();

        while(opModeIsActive()){


        telemetry.addData("red", color.red());
        telemetry.addData("blue", color.blue());
        telemetry.addData("green", color.green());
        telemetry.addData("auto", auto);
        telemetry.addData("power", power);

        telemetry.update();


        }


    }

    private void driveUpdate() {
        if (gamepad1.right_bumper) { // replace this with a button for sprint
            drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
        } else {
            drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
        }
    }

    private void intakeUpdate(){
        if (gamepad1.a || (color.red() > 130 && color.green() > 150 && color.blue() > 60)) {
            intake.clampAndRelease(true);
        } else {
            intake.clampAndRelease(false);
        }

        if(gamepad1.left_bumper){
            auto = !auto;

        }

        if(gamepad1.start){
            if(changePower) {
                changePower();
                changePower = false;
            }
        }else{
            changePower = true;
        }

        if(!auto) {

            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(true, gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                intake.setPower(false, gamepad1.left_trigger);
            } else {
                intake.setPower(true, 0);
            }
        }else{
            if(gamepad1.right_trigger > 0.1){
                intake.setPower(true, power);
            }else if(gamepad1.left_trigger > 0.1){
                intake.setPower(false, power);
            }else{
                intake.setPower(true, 0);
            }
        }


    }


    public void changePower(){
        if(power == 0.3){
            power = 0.5;
        }else if(power == 0.5){
            power = 0.7;
        }else if(power == 0.7){
            power = 1;
        }else{
            power = 0.3;
        }
    }
}
