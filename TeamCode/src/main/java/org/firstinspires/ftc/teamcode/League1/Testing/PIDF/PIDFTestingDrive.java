package org.firstinspires.ftc.teamcode.League1.Testing.PIDF;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    BNO055IMU imu;

    //For Rotate method (tankRotatePID)
    PIDCoefficients pid = new PIDCoefficients(0, 0, 0);


    @Override
    public void runOpMode() throws InterruptedException {
        constants = new Constants();
        drive = new MecDrive(hardwareMap, true, telemetry);
        score = new ScoringSystem2(hardwareMap, constants, telemetry);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        drive.goTOPIDPos(500, 0.5, MecDrive.MovementType.STRAIGHT);
        sleep(1000);
        tankRotatePID(Math.PI, 0.5);

        while(opModeIsActive()){

        }

        drive.simpleBrake();
    }


    public void tankRotatePID(double radians, double power){

        /*if(radians > imu.getAngularOrientation().firstAngle){
            power *= -1;
        }*/

        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();

        radians = wrapAngle(radians);
        double radError = wrapAngle(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - radians);
        double previousError = radError;
        double integralSum = 0;


        while(Math.abs(radError) > 0.0001){

            telemetry.addData("target", radians);

            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

            double currentTime = time.seconds();

            radError = wrapAngle(currentAngle - radians);
            telemetry.addData("Error", radError);

            integralSum += (radError + previousError)/(currentTime - startTime);
            telemetry.addData("Integral", integralSum);

            //TODO:See if we need an integral limit

            double derivative = (radError - previousError)/(currentTime - startTime);
            telemetry.addData("Derivative", derivative);

            drive.setPowerAuto(((pid.p * radError) + (pid.i * integralSum) + (pid.d * derivative)), MecDrive.MovementType.ROTATE);

            startTime = currentTime;
            previousError = radError;
            telemetry.update();

        }

        drive.simpleBrake();




    }

    public double wrapAngle(double angle){
        while(angle > Math.PI){
            angle -= (2 * Math.PI);
        }

        while(angle < -Math.PI){
            angle += (2 * Math.PI);
        }

        return angle;
    }




}
