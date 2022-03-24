package org.firstinspires.ftc.teamcode.MecanumCode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.MecanumDriveTrainCopyPID;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.OpModeWrapper;
import org.firstinspires.ftc.teamcode.Testing.TalonsMecanumDriveTrain;

import java.io.FileNotFoundException;


@Autonomous(name="PID Tester")
public class TestAuto extends OpModeWrapper {

    //MecanumDriveTrain drive;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    MecanumDriveTrainCopyPID drive;

    Thread telemetryThread;








    @Override
    protected void onInitialize() throws FileNotFoundException {
        drive = new MecanumDriveTrainCopyPID(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //drive = new TalonsMecanumDriveTrain(hardwareMap);


        telemetryThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()) {
                    telemetry.addData("fr", drive.fr.getVelocity());
                    telemetry.addData("fl", drive.fl.getVelocity());
                    telemetry.addData("br", drive.br.getVelocity());
                    telemetry.addData("bl", drive.bl.getVelocity());
                    telemetry.addData("avg velocity", drive.avgVel());
                    telemetry.addData("fr tics", drive.fr.getCurrentPosition());
                    telemetry.update();
                }


            }
        };





    }

    @Override
    protected void onStart() {
        telemetryThread.start();
        drive.PID(500, 2000);

    }

    @Override
    protected void onStop() {

    }



}
