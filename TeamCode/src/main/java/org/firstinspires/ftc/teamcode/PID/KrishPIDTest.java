package org.firstinspires.ftc.teamcode.PID;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.OpModeWrapper;

import java.io.FileNotFoundException;

@Autonomous(name = "KRISH PID")
public class KrishPIDTest extends OpModeWrapper {

    MecanumDriveTrain drive;

    double integralSum = 0;
    double Kp = 0.19;
    double Ki = 0.015;
    double Kd = 0.1;

    boolean stillRun;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    protected void onInitialize() throws FileNotFoundException {
        drive = new MecanumDriveTrain(hardwareMap, true);
        stillRun = true;


    }

    @Override
    protected void onStart() {

        goToPosition(1000);
        drive.tankRotate(-Math.PI/2, 0.3);
        goToPosition(1000);

        while (opModeIsActive()) {
            // backwards
            /*double flPow = PIDControl(500, drive.fl.getCurrentPosition());
            double frPow = PIDControl(-500, drive.fr.getCurrentPosition());
            double blPow = PIDControl(500, drive.bl.getCurrentPosition());
            double brPow = PIDControl(-500, drive.br.getCurrentPosition());
            telemetry.addData("fl ", drive.fl.getCurrentPosition());
            telemetry.addData("fr ", drive.fr.getCurrentPosition());
            telemetry.addData("bl ", drive.bl.getCurrentPosition());
            telemetry.addData("br ", drive.br.getCurrentPosition());
            telemetry.update();

            drive.setPower(flPow, -frPow, blPow, -brPow);
            double flPow = PIDControl(500, drive.fl.getCurrentPosition());
            double frPow = PIDControl(-500, drive.fr.getCurrentPosition());
            double blPow = PIDControl(500, drive.bl.getCurrentPosition());
            double brPow = PIDControl(-500, drive.br.getCurrentPosition());

             */
            telemetry.addData("fl ", drive.fl.getCurrentPosition());
            telemetry.addData("fr ", drive.fr.getCurrentPosition());
            telemetry.addData("bl ", drive.bl.getCurrentPosition());
            telemetry.addData("br ", drive.br.getCurrentPosition());
            telemetry.update();

            //drive.setPower(flPow, -frPow, blPow, -brPow);



        }
    }

    @Override
    protected void onStop() {

    }

    public void goToPosition(int position){
        drive.setPower(PIDControl(position, drive.fl.getCurrentPosition()), PIDControl(-position, drive.fr.getCurrentPosition()),
                PIDControl(position, drive.bl.getCurrentPosition()), PIDControl(-position, drive.br.getCurrentPosition()));
    }

    public double PIDControl(double reference, double state) {

        if (stillRun) {

            double error = reference - state;
            integralSum += error * timer.seconds();
            double derivative = (error - lastError) / timer.seconds();

            timer.reset();

            double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
            lastError = error;
        /*
        telemetry.addData("error ", error);
        telemetry.addData("derivative ", derivative);
        telemetry.addData("integralSum ", integralSum);

         */

            if(Math.abs(state) > Math.abs(reference)){
                stillRun = false;
            }


            return output;

        }


        return 0;
    }

}
