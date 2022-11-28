package org.firstinspires.ftc.teamcode.League1.Testing.PIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
@Config
public class PIDFTestingDrive extends LinearOpMode {
    MecDrive drive;
    ScoringSystem2 score;
    Constants constants;

    public static double target = 1.57;
    public static double p = 0, i = 0, d = 0;

    int flPreviousError = 0;
    int frPreviousError = 0;
    int blPreviousError = 0;
    int brPreviousError = 0;
    double previousError = 0;

    int flIntegralSum = 0;
    int frIntegralSum = 0;
    int blIntegralSum = 0;
    int brIntegralSum = 0;
    double integralSum = 0;


    double currentTime;
    double startTime;
    ElapsedTime time = new ElapsedTime();


    //For Rotate method (tankRotatePID)


    @Override
    public void runOpMode() throws InterruptedException {
        constants = new Constants();
        drive = new MecDrive(hardwareMap, true, telemetry, hardwareMap.get(ColorRangeSensor.class, "color"));
        score = new ScoringSystem2(hardwareMap, constants, telemetry);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();
        startTime = time.seconds();


        //Straight, Strafe, Encoder Rotate
        //TODO: return constants
        //drive.goTOPIDPos(-3000, 0.5, MecDrive.MovementType.STRAIGHT);

        //drive.autoDiagonals(true);


        //drive.goTOPIDPosAvg(3000, 1, MecDrive.MovementType.STRAIGHT);
        //sleep(1000);
        //drive.goTOPIDPos(-2120, 1,MecDrive.MovementType.STRAIGHT);

        //drive.goTOPIDPos(-250, 1, MecDrive.MovementType.STRAFE);0/


        //IMU Rotate
        //drive.tankRotatePID(Math.PI, 0.85);

        while (opModeIsActive()) {

            tankRotatePID(target, 1, false);


            /*telemetry.addData("flPos", drive.getFLEncoder());
            telemetry.addData("frPos", drive.getFREncoder());
            telemetry.addData("blPos", drive.getBLEncoder());
            telemetry.addData("brPos", drive.getBREncoder());
*/

            telemetry.addData("current", drive.getFirstAngle());
            telemetry.addData("target", target);

            telemetry.update();

        }

        drive.simpleBrake();
    }


    public void goTOPIDPos(int tics, double power, MecDrive.MovementType movement) {


        //TODO: check if we need to negate any

        currentTime = time.seconds();


        //TODO: check if we need to negate any
        int flPos = -1 * drive.getFLEncoder();
        int frPos = drive.getFREncoder();
        int blPos = -1 * drive.getBLEncoder();
        int brPos = drive.getBREncoder();


        int flError = tics - flPos;
        int frError = tics - frPos;
        int blError = tics - blPos;
        int brError = tics - brPos;


        flIntegralSum += (0.5 * (flError + flPreviousError) * (currentTime - startTime));
        frIntegralSum += (0.5 * (frError + frPreviousError) * (currentTime - startTime));
        blIntegralSum += (0.5 * (blError + blPreviousError) * (currentTime - startTime));
        brIntegralSum += (0.5 * (brError + brPreviousError) * (currentTime - startTime));


        //TODO: look at telemetry and see if we can have new bound (change integral sum limit)
        if (flIntegralSum > 20000) {
            flIntegralSum = 20000;
        } else if (flIntegralSum < -20000) {
            flIntegralSum = -20000;
        }

        if (frIntegralSum > 20000) {
            frIntegralSum = 20000;
        } else if (frIntegralSum < -20000) {
            frIntegralSum = -20000;
        }

        if (blIntegralSum > 20000) {
            blIntegralSum = 20000;
        } else if (blIntegralSum < -20000) {
            blIntegralSum = -20000;
        }

        if (brIntegralSum > 20000) {
            brIntegralSum = 20000;
        } else if (brIntegralSum < -20000) {
            brIntegralSum = -20000;
        }


        double flDerivative = (flError - flPreviousError) / (currentTime - startTime);
        double frDerivative = (frError - frPreviousError) / (currentTime - startTime);
        double blDerivative = (blError - blPreviousError) / (currentTime - startTime);
        double brDerivative = (brError - brPreviousError) / (currentTime - startTime);


        double flPower = ((p * flError) + (i * flIntegralSum) + (d * flDerivative));
        double frPower = ((p * frError) + (i * frIntegralSum) + (d * frDerivative));
        double blPower = ((p * blError) + (i * blIntegralSum) + (d * blDerivative));
        double brPower = ((p * brError) + (i * brIntegralSum) + (d * brDerivative));


        //TODO: Fix rotate and check Strafe
        drive.setPower(flPower, frPower, blPower, brPower);


        startTime = currentTime;
        flPreviousError = flError;
        frPreviousError = frError;
        blPreviousError = blError;
        brPreviousError = brError;


    }


    public void tankRotatePID(double radians, double power, boolean slidesUp) {

        radians = drive.wrapAngle(radians);

        currentTime = time.seconds();

        double radError = drive.wrapAngle(drive.getFirstAngle() - radians);


        integralSum += (radError + previousError) / (currentTime - startTime);

        //TODO:See if we need an integral limit
        if (integralSum > 10000) {
            integralSum = 10000;
        } else if (integralSum < -10000) {
            integralSum = -10000;
        }

        double derivative = (radError - previousError) / (currentTime - startTime);


        drive.setPowerAuto(((p * radError) + (i * integralSum) + (d * derivative)), MecDrive.MovementType.ROTATE);


        startTime = currentTime;
        previousError = radError;


    }


}
