package org.firstinspires.ftc.teamcode.League1.Testing.PIDF;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;

@Autonomous
public class PIDFTestingDrive extends LinearOpMode {
    MecDrive drive;
    ScoringSystem2 score;
    Constants constants;


    //For Rotate method (tankRotatePID)


    @Override
    public void runOpMode() throws InterruptedException {
        constants = new Constants();
        drive = new MecDrive(hardwareMap, true, telemetry, hardwareMap.get(ColorRangeSensor.class, "color"));
        score = new ScoringSystem2(hardwareMap, constants, telemetry);



        waitForStart();

        //Straight, Strafe, Encoder Rotate
        //TODO: return constants
        //drive.goTOPIDPos(-3000, 0.5, MecDrive.MovementType.STRAIGHT);

        //drive.autoDiagonals(true);


        //drive.goTOPIDPosAvg(3000, 1, MecDrive.MovementType.STRAIGHT);
        //sleep(1000);
        //drive.goTOPIDPos(-2120, 1,MecDrive.MovementType.STRAIGHT);

        //drive.goTOPIDPos(-250, 1, MecDrive.MovementType.STRAFE);0/
        drive.tankRotatePID(Math.PI/4, 1, false);
        sleep(1000);
        drive.tankRotatePID(Math.PI/2, 1, false);


        //IMU Rotate
        //drive.tankRotatePID(Math.PI, 0.85);

        while(opModeIsActive()){
            telemetry.addData("fl", drive.getFLEncoder());
            telemetry.addData("fr", drive.getFREncoder());
            telemetry.addData("bl", drive.getBLEncoder());
            telemetry.addData("br", drive.getBREncoder());
            telemetry.addData("first Angle", drive.getFirstAngle());
            telemetry.update();

        }

        drive.simpleBrake();
    }




}
