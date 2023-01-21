package org.firstinspires.ftc.teamcode.Testing.PIDF;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Used.ScoringSystem2;

import java.util.concurrent.atomic.AtomicBoolean;

@Disabled
@Autonomous
public class IDTestingLift extends LinearOpMode {
    Thread idController;
    AtomicBoolean hold;
    ScoringSystem2 score;
    //Constants constants;
    PIDCoefficients pid = new PIDCoefficients(0, 0.000025, 0.0008);
    int[] intSums;


    @Override
    public void runOpMode() throws InterruptedException {

        hold = new AtomicBoolean(false);
        //constants = newConstants();
        score = new ScoringSystem2(hardwareMap, telemetry);


        idController = new Thread(){
            @Override
            public void run() {



                while(opModeIsActive()){
                    if(hold.get()){
                        ElapsedTime time = new ElapsedTime();
                        double startTime = time.seconds();

                        int leftIntegralSum = intSums[0];
                        int rightIntegralSum = intSums[1];


                        int rLiftPos = score.getRightEncoderPos();
                        int lLiftPos = -1 * score.getLeftEncoderPos();

                        int tics = 850;

                        int leftPreviousError = Math.abs(tics - lLiftPos);
                        int rightPreviousError = Math.abs(tics - rLiftPos);

                        while(hold.get()){
                            telemetry.addData("target", tics);


                            rLiftPos = score.getRightEncoderPos();
                            lLiftPos = -1 * score.getLeftEncoderPos();

                            telemetry.addData("rLift", rLiftPos);
                            telemetry.addData("lLift", lLiftPos);


                            double currentTime = time.seconds();

                            int leftError = tics - lLiftPos;
                            int rightError = tics - rLiftPos;

                            telemetry.addData("leftError", leftError);
                            telemetry.addData("rightError", rightError);


                            leftIntegralSum += (0.5 * (leftError + leftPreviousError) * (currentTime - startTime));
                            rightIntegralSum += (0.5 * (rightError + rightPreviousError) * (currentTime - startTime));

                            telemetry.addData("rightIntegral", rightIntegralSum);
                            telemetry.addData("leftIntegral", leftIntegralSum);


                            //TODO: look at telemetry and see if we can have new bound (change integral sum limit)
                            if(leftIntegralSum > 20000){
                                leftIntegralSum = 20000;
                            }else if(leftIntegralSum < -20000){
                                leftIntegralSum = -20000;
                            }

                            if(rightIntegralSum > 20000){
                                rightIntegralSum = 20000;
                            }else if(rightIntegralSum < -20000){
                                rightIntegralSum = -20000;
                            }



                            double leftDerivative = (leftError - leftPreviousError)/(currentTime - startTime);
                            double rightDerivative = (rightError - rightPreviousError)/(currentTime - startTime);

                            telemetry.addData("rightDerivative", rightDerivative);
                            telemetry.addData("leftDerivative", leftDerivative);

                            double leftPower = (pid.i * leftIntegralSum) + (pid.d * leftDerivative);
                            double rightPower = (pid.i * rightIntegralSum) + (pid.d * rightDerivative);

                            telemetry.addData("rightPower", rightPower);
                            telemetry.addData("leftPower", leftPower);

                            if(tics < ((rLiftPos + lLiftPos) / 2)){
                                leftPower *= -1;
                                rightPower *= -1;
                            }

                            score.setPower(rightPower, leftPower);







                            startTime = currentTime;
                            leftPreviousError = leftError;
                            rightPreviousError = rightError;

                            telemetry.update();



                        }




                    }
                }
            }
        };




        waitForStart();
        idController.start();

        intSums = score.moveToPosition(850  , 1, false);
        hold.set(true);
        while(opModeIsActive()){

        }
        hold.set(false);
    }
}
