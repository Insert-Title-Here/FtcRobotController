package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.Testing.RobotK;

import java.io.FileNotFoundException;

@Autonomous(name = "StuffAuto")
public class TestingAuto extends OpModeWrapper {

    //RobotK robot;
    //MecanumDriveTrain drive;
    LynxModule hub;

    DcMotorEx fl, fr, bl, br;


    @Override
    protected void onInitialize() throws FileNotFoundException {
        //robot = new RobotK(hardwareMap, telemetry);
        //drive = new MecanumDriveTrain(hardwareMap);
        hub = hardwareMap.get(LynxModule.class, "Control Hub");
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);




        fl = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        br = hardwareMap.get(DcMotorEx.class, "BackRightDrive");





    }

    @Override
    protected void onStart() {
        //robot.drive.driveAuto(0.5, 50, MecanumDriveTrain.MovementType.STRAIGHT, telemetry);
        int[] bulkEncoders = new int[4];

        goToPosition(200, 20);

        //robot.drive.driveAuto(0.5, 100, MecanumDriveTrain.MovementType.STRAIGHT);

        while(opModeIsActive()){
            bulkEncoders = getEncoderValues();

            /*
            telemetry.addData("fl ", drive.fl.getCurrentPosition());
            telemetry.addData("fr ", drive.fr.getCurrentPosition());
            telemetry.addData("bl ", drive.bl.getCurrentPosition());
            telemetry.addData("br ", drive.br.getCurrentPosition());



            telemetry.addData("fl ", robot.drive.fl.getCurrentPosition());
            telemetry.addData("fr ", robot.drive.fr.getCurrentPosition());
            telemetry.addData("bl ", robot.drive.bl.getCurrentPosition());
            telemetry.addData("br ", robot.drive.br.getCurrentPosition());

             */


            telemetry.addData("fl bulk ", bulkEncoders[0]);
            telemetry.addData("fr bulk", bulkEncoders[1]);
            telemetry.addData("bl bulk", bulkEncoders[2]);
            telemetry.addData("br bulk", bulkEncoders[3]);

            telemetry.update();

        }




/*
        robot.drive.fl.setTargetPosition(-1000);
        robot.drive.fr.setTargetPosition(-1000);
        robot.drive.bl.setTargetPosition(1000);
        robot.drive.br.setTargetPosition(1000);




        robot.drive.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.drive.setPower(0.5,0.5,0.5,0.5);

        while(robot.drive.fl.isBusy() || robot.drive.fr.isBusy() || robot.drive.bl.isBusy() || robot.drive.br.isBusy()){
            telemetry.addData("fl ", robot.drive.fl.getCurrentPosition());
            telemetry.addData("fr ", robot.drive.fr.getCurrentPosition());
            telemetry.addData("bl ", robot.drive.bl.getCurrentPosition());
            telemetry.addData("br ", robot.drive.br.getCurrentPosition());
            telemetry.update();

        }

 */




    }


    @Override
    protected void onStop() {

    }

    public int[] getEncoderValues(){

        LynxModule.BulkData data = hub.getBulkData();

        int[] encoderValues = {data.getMotorCurrentPosition(0), data.getMotorCurrentPosition(1), data.getMotorCurrentPosition(2), data.getMotorCurrentPosition(3)};
        hub.clearBulkCache();

        return encoderValues;


    }

    //TODO: fix this (went the wrong way, maybe change negative to positive or vice versa velocity)
    public void goToPosition(int targetPosition, int velocity){

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fl.setTargetPosition(-targetPosition);
        fr.setTargetPosition(-targetPosition);
        bl.setTargetPosition(targetPosition);
        br.setTargetPosition(targetPosition);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setVelocity(velocity, AngleUnit.DEGREES);
        fr.setVelocity(velocity, AngleUnit.DEGREES);
        bl.setVelocity(velocity, AngleUnit.DEGREES);
        br.setVelocity(velocity, AngleUnit.DEGREES);

        while((fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()) && opModeIsActive()){

        }
        fl.setVelocity(0, AngleUnit.DEGREES);
        fr.setVelocity(0, AngleUnit.DEGREES);
        bl.setVelocity(0, AngleUnit.DEGREES);
        br.setVelocity(0, AngleUnit.DEGREES);




    }



}
