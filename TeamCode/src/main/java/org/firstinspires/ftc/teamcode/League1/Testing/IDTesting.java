package org.firstinspires.ftc.teamcode.League1.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;

import java.util.concurrent.atomic.AtomicBoolean;

public class IDTesting extends LinearOpMode {
    Thread idController;
    AtomicBoolean hold;
    ScoringSystem2 score;
    Constants constants;
    PIDCoefficients pid = new PIDCoefficients(0, 0.0001, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        hold = new AtomicBoolean(false);
        constants = new Constants();
        score = new ScoringSystem2(hardwareMap, constants);


        idController = new Thread(){
            @Override
            public void run() {



                while(opModeIsActive()){
                    if(hold.get()){
                        ElapsedTime time = new ElapsedTime();
                        double startTime = time.milliseconds();

                        int leftIntegralSum = 0;
                        int rightIntegralSum = 0;


                        int rLiftPos = score.getRightEncoderPos();
                        int lLiftPos = -1 * score.getLeftEncoderPos();

                        int tics = score.getHeight();

                        int leftPreviousError = Math.abs(tics - lLiftPos);
                        int rightPreviousError = Math.abs(tics - rLiftPos);

                        while(hold.get()){

                            rLiftPos = score.getRightEncoderPos();
                            lLiftPos = -1 * score.getLeftEncoderPos();

                            double currentTime = time.milliseconds();

                            int leftError = tics - lLiftPos;
                            int rightError = tics - rLiftPos;

                            leftIntegralSum += (0.5 * (leftError + leftPreviousError) * (currentTime - startTime));
                            rightIntegralSum += (0.5 * (rightError + rightPreviousError) * (currentTime - startTime));

                            if(leftIntegralSum > 2000){
                                leftIntegralSum = 2000;
                            }

                            if(rightIntegralSum > 2000){
                                rightIntegralSum = 2000;
                            }

                            double leftDerivative = (leftError - leftPreviousError)/(currentTime - startTime);
                            double rightDerivative = (rightError - rightPreviousError)/(currentTime - startTime);

                            double leftPower = (pid.i * leftIntegralSum) + (pid.d * leftDerivative);
                            double rightPower = (pid.i * rightIntegralSum) + (pid.d * rightDerivative);

                            if(tics < ((rLiftPos + lLiftPos) / 2)){
                                leftPower *= -1;
                                rightPower *= -1;
                            }

                            score.setPower(rightPower, leftPower);


                            telemetry.addData("rightLiftPosition", rLiftPos);
                            telemetry.addData("leftLiftPosition", lLiftPos);
                            telemetry.addData("rightError", rightError);
                            telemetry.addData("leftError", leftError);
                            telemetry.addData("rightIntegralSum", rightIntegralSum);
                            telemetry.addData("leftIntegralSum", leftIntegralSum);
                            telemetry.addData("rightDerivative", rightDerivative);
                            telemetry.addData("leftDerivative", leftDerivative);
                            telemetry.update();






                            startTime = currentTime;
                            leftPreviousError = leftError;
                            rightPreviousError = rightError;



                        }




                    }
                }
            }
        };




        waitForStart();
        idController.start();

        score.autoGoToPosition(ScoringSystem2.ScoringMode.LOW);

        sleep(1000);
        hold.set(true);
        while(opModeIsActive()){

        }
    }
}
