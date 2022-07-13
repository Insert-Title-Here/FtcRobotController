package org.firstinspires.ftc.teamcode.TalonTester;



import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.teamcode.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.OpModeWrapper;

import java.io.FileNotFoundException;

@TeleOp(name = "Stuff")
public class TestingTeleOp extends LinearOpMode {

    MecanumDriveTrain drive;
    ColorRangeSensor frontRed;


    @Override
    public void runOpMode() throws InterruptedException {

        try {
            drive = new MecanumDriveTrain(hardwareMap);
            frontRed = hardwareMap.get(ColorRangeSensor.class, "frontRed");
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }




        waitForStart();

        while(opModeIsActive()){
            telemetry.addData(gamepad1.left_stick_x + ", " + gamepad1.left_stick_y, "");
            telemetry.update();
        }


    }
}
