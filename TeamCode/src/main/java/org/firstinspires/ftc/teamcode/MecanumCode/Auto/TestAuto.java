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
import org.firstinspires.ftc.teamcode.MecanumCode.Common.OpModeWrapper;
import org.firstinspires.ftc.teamcode.Testing.TalonsMecanumDriveTrain;

import java.io.FileNotFoundException;

@Disabled
@Autonomous(name="Test Auto")
public class TestAuto extends OpModeWrapper {

    //MecanumDriveTrain drive;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    TalonsMecanumDriveTrain drive;







    @Override
    protected void onInitialize() throws FileNotFoundException {
        //drive = new MecanumDriveTrain(hardwareMap);

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
        drive = new TalonsMecanumDriveTrain(hardwareMap);


    }

    @Override
    protected void onStart() {
        /*drive.driveAuto(120, 240, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(120, 240, MecanumDriveTrain.MovementType.STRAFE);
        drive.driveAuto(120, 240, MecanumDriveTrain.MovementType.ROTATE);

         */
        // Forward: 1 ft 540.3 tics (5403 for 10 ft)
        // Rotation: 360 degrees 3665 tics
        // Strafe: 590 tics/ft

        /*
        drive.driveAuto(0.3, 540, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(0.5, -590, MecanumDriveTrain.MovementType.STRAFE);
        drive.driveAuto(0.3, 3665, MecanumDriveTrain.MovementType.ROTATE);
        drive.driveAuto(0.7, -3665, MecanumDriveTrain.MovementType.ROTATE);
        drive.driveAuto(0.2, -540, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(0.3, 590, MecanumDriveTrain.MovementType.STRAFE);

         */



        //drive.driveAuto(1, 1000, MecanumDriveTrain.MovementType.STRAIGHT);
        //double angle = imu.getAngularOrientation().firstAngle;
        //drive.writeLoggerToFile();
/*
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle < Math.PI){
            drive.setPowerAuto(0.3, MecanumDriveTrain.MovementType.ROTATE);

            telemetry.addData("Radians: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS).firstAngle);
            telemetry.update();

        }

 */


        //moveTest(3000);
        /*
        drive.setPower(0.5, 0, 0, -0.5);
        sleep(1000);
        drive.setPower(-0.5, 0, 0, 0.5);
        sleep(1000);
        drive.setPower(0,0,0,0);


         */
    }

    @Override
    protected void onStop() {

    }


    /*private void moveTest(int motorTics){


        for(DcMotor motor: drive.getMotors()){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(motorTics);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }



        drive.setPowerAuto(0.5, MecanumDriveTrain.MovementType.STRAIGHT);

        while(drive.fr.getCurrentPosition() < motorTics){
            telemetry.addData("FL Tics", drive.fl.getCurrentPosition());
            telemetry.addData("FR Tics", drive.fr.getCurrentPosition());
            telemetry.addData("BL Tics", drive.bl.getCurrentPosition());
            telemetry.addData("BR Tics", drive.br.getCurrentPosition());
            telemetry.update();

        }


        drive.setPowerAuto(0, MecanumDriveTrain.MovementType.STRAIGHT);


    }*/
}
