package org.firstinspires.ftc.teamcode.Testing.PIDF;

////import com.acmerobotics.dashboard.FtcDashboard;
////import com.acmerobotics.dashboard.config.Config;
////import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.ScoringSystemV2EpicLift;

@Disabled
@Autonomous
@Config
public class PIDFTestingLiftVel extends LinearOpMode {

    ScoringSystemV2EpicLift score;
    //Constants constants;

    public static boolean pid = true;



    public static int target = 300;
    public static double p = 0.0085, i = 0, d = 0.00023;

    int rightPreviousError = 0;
    int leftPreviousError = 0;

    int rightIntegralSum = 0;
    int leftIntegralSum = 0;


    double currentTime;
    double startTime;
    ElapsedTime time = new ElapsedTime();


    //For Rotate method (tankRotatePID)


    @Override
    public void runOpMode() throws InterruptedException {
        //constants = newConstants();
        score = new ScoringSystemV2EpicLift(hardwareMap, telemetry, false);

        score.setLinkagePosition(Constants.linkageUpV2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        boolean thing = true;
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

            if(pid) {

                newLiftPID(target);
                thing = true;
            }else{



                if(thing){
                    rightIntegralSum = 0;
                    leftIntegralSum = 0;
                    leftPreviousError = 0;
                    rightPreviousError = 0;
                    thing = false;
                }
                newLiftID(target);
            }

            telemetry.addData("rightIntegral", rightIntegralSum);
            telemetry.addData("leftIntegral",leftIntegralSum);
            telemetry.addData("rightPos", -1 * score.getRightEncoderPos());
            telemetry.addData("leftPos", -1 * score.getLeftEncoderPos());






            telemetry.update();

        }
        score.setPower(0);
    }


    public void newLiftPID(int tics) {
        currentTime = time.seconds();


        //TODO: check if we need to negate any

        int rightPos = -1 * score.getRightEncoderPos();
        int leftPos = -1 * score.getLeftEncoderPos();

        int rightError = tics - rightPos;
        int leftError = tics - leftPos;



        //TODO: check if we need to negate any





        double rightDerivative = (rightError - rightPreviousError) / (currentTime - startTime);
        double leftDerivative = (leftError - leftPreviousError) / (currentTime - startTime);


        double rightPower = ((p * rightError) + (d * rightDerivative));
        double leftPower = ((p * leftError)+ (d * leftDerivative));


        score.setVelocity(rightPower, leftPower);


        startTime = currentTime;
        rightPreviousError = rightError;
        leftPreviousError = leftError;


        telemetry.update();


    }


    public void newLiftID(int tics) {
        currentTime = time.seconds();


        //TODO: check if we need to negate any

        int rightPos = -1 * score.getRightEncoderPos();
        int leftPos = score.getLeftEncoderPos();

        int rightError = tics - rightPos;
        int leftError = tics - leftPos;






        //TODO: check if we need to negate any


        rightIntegralSum += (50 * (rightError + rightPreviousError) * (currentTime - startTime));
        leftIntegralSum += (50 * (leftError + leftPreviousError) * (currentTime - startTime));




        //TODO: look at telemetry and see if we can have new bound (change integral sum limit)
        if (rightIntegralSum > 20000) {
            rightIntegralSum = 20000;
        } else if (rightIntegralSum < -20000) {
            rightIntegralSum = -20000;
        }

        if (leftIntegralSum > 20000) {
            leftIntegralSum = 20000;
        } else if (leftIntegralSum < -20000) {
            leftIntegralSum = -20000;
        }


        double rightDerivative = (rightError - rightPreviousError) / (currentTime - startTime);
        double leftDerivative = (leftError - leftPreviousError) / (currentTime - startTime);


        double rightPower = ((i * rightIntegralSum));
        double leftPower = ((i * leftIntegralSum));


        score.setPower(rightPower, leftPower);


        startTime = currentTime;
        rightPreviousError = rightError;
        leftPreviousError = leftError;


        telemetry.update();


    }




}
