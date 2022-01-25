package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumCode.Common.Vector2D;

public class TalonsMecanumDriveTrain{


        private BNO055IMU imu;
        private DcMotorEx fl, fr, bl, br;
        LynxModule hub;
        Vector2D previousVelocity;
        double previousOmega;


    public TalonsMecanumDriveTrain(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);


        fl = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        br = hardwareMap.get(DcMotorEx.class, "BackRightDrive");


        hub = hardwareMap.get(LynxModule.class, "Control Hub");
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);




        previousVelocity = new Vector2D(0, 0);
        previousOmega = 0;
        correctMotors();

    }

    public void rotateDistanceDERadian(double radians, double power) {
        double deltaRadians = radians - imu.getAngularOrientation().firstAngle;
        power *= getSign(deltaRadians);
        while (Math.abs(imu.getAngularOrientation().firstAngle - radians) > 0.05) {
            setPower(power, -power, power, -power);
        }
        brake();
    }

    public void rotateDistanceDE(double degrees, double power) {
        double radians = Math.toRadians(degrees);
        rotateDistanceDERadian(radians, power);
    }

    /**
     * @param distance tics
     * @param degrees  degrees
     * @param power    voltage, always should be positive
     * @param omega    voltage, always should be positive
     */
    public void moveDistanceDE(int distance, double degrees, double power, double omega) {
        double radians = Math.toRadians(degrees);
        power *= getSign(distance);
        Vector2D vec = Vector2D.fromAngleMagnitude(radians, power);
        double globalHeading = imu.getAngularOrientation().firstAngle;
        radians = radians + (Math.PI / 4.0); //45deg + globalHeading
        int flDistance = (int) (Math.sin(radians) * distance); //TODO check if this is right, if not flip the sign
        int frDistance = (int) (Math.cos(radians) * distance);
        int blDistance = (int) (Math.cos(radians) * distance);
        int brDistance = (int) (Math.sin(radians) * distance);
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);


        LynxModule.BulkData data = hub.getBulkData();
        //AbstractOpMode.currentOpMode().telemetry.addData("fl", data.getMotorCurrentPosition(0));
        AbstractOpMode.currentOpMode().telemetry.addData("fl", flDistance);
        //AbstractOpMode.currentOpMode().telemetry.addData("fr", data.getMotorCurrentPosition(1));
        AbstractOpMode.currentOpMode().telemetry.addData("fr", frDistance);
        //AbstractOpMode.currentOpMode().telemetry.addData("bl", data.getMotorCurrentPosition(2));
        AbstractOpMode.currentOpMode().telemetry.addData("bl", blDistance);
        //AbstractOpMode.currentOpMode().telemetry.addData("br", data.getMotorCurrentPosition(3));
        AbstractOpMode.currentOpMode().telemetry.addData("br", brDistance);
        AbstractOpMode.currentOpMode().telemetry.update();

        while ((-Math.abs(data.getMotorCurrentPosition(0)) < Math.abs(flDistance) && Math.abs(data.getMotorCurrentPosition(1)) < Math.abs(frDistance)
                && -Math.abs(data.getMotorCurrentPosition(2)) < Math.abs(blDistance) && Math.abs(data.getMotorCurrentPosition(3)) < Math.abs(brDistance))) {
            hub.clearBulkCache();
            data = hub.getBulkData();
            AbstractOpMode.currentOpMode().telemetry.addData("fl", -data.getMotorCurrentPosition(0));
            //AbstractOpMode.currentOpMode().telemetry.addData("fl", flDistance);
            AbstractOpMode.currentOpMode().telemetry.addData("fr", data.getMotorCurrentPosition(1));
            //AbstractOpMode.currentOpMode().telemetry.addData("fr", frDistance);
            AbstractOpMode.currentOpMode().telemetry.addData("bl", -data.getMotorCurrentPosition(2));
            //AbstractOpMode.currentOpMode().telemetry.addData("bl", blDistance);
            AbstractOpMode.currentOpMode().telemetry.addData("br", data.getMotorCurrentPosition(3));
            //AbstractOpMode.currentOpMode().telemetry.addData("br", brDistance);
            AbstractOpMode.currentOpMode().telemetry.update();


            setPower(vec, 0);
        }

        brake();
        rotateDistanceDERadian(globalHeading, omega);
    }

    public void drive(int distance, double power){
        while((hub.getBulkData().getMotorCurrentPosition(0) + hub.getBulkData().getMotorCurrentPosition(0)
                + hub.getBulkData().getMotorCurrentPosition(0) + hub.getBulkData().getMotorCurrentPosition(0) < distance)) {
            setPower(power, power, power, power);

            LynxModule.BulkData data = hub.getBulkData();
            AbstractOpMode.currentOpMode().telemetry.addData("fl", -data.getMotorCurrentPosition(0));
            //AbstractOpMode.currentOpMode().telemetry.addData("fl", flDistance);
            AbstractOpMode.currentOpMode().telemetry.addData("fr", data.getMotorCurrentPosition(1));
            //AbstractOpMode.currentOpMode().telemetry.addData("fr", frDistance);
            AbstractOpMode.currentOpMode().telemetry.addData("bl", -data.getMotorCurrentPosition(2));
            //AbstractOpMode.currentOpMode().telemetry.addData("bl", blDistance);
            AbstractOpMode.currentOpMode().telemetry.addData("br", data.getMotorCurrentPosition(3));
            //AbstractOpMode.currentOpMode().telemetry.addData("br", brDistance);
            AbstractOpMode.currentOpMode().telemetry.update();
        }
    }

    public void setEncoderMode(DcMotor.RunMode runMode) {
        fl.setMode(runMode);
        fr.setMode(runMode);
        bl.setMode(runMode);
        br.setMode(runMode);
    }

    private int getSign(double num){
        if(num < 0){
            return -1;
        }else {
            return 1;
        }
    }

    private void brake() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        previousVelocity = new Vector2D(0,0);
    }

    private void correctMotors() {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Vector2D setPower(Vector2D velocity, double turnValue){
        turnValue = -turnValue;
        double direction = velocity.getDirection();



        double power = velocity.magnitude();

        double angle = direction + ( Math.PI / 4.0);
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        setPower((power * sin - turnValue),(power * cos + turnValue),
                (power * cos - turnValue), (power * sin + turnValue));
        return new Vector2D(velocity.getX(), velocity.getY());
    }

    public Vector2D setPower(Vector2D velocity, double turnValue, double robotHeading){
        turnValue = -turnValue;
        double direction = velocity.getDirection() + robotHeading;



        double power = velocity.magnitude();

        double angle = direction + ( Math.PI / 4.0);
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        setPower((power * sin - turnValue),(power * cos + turnValue),
                (power * cos - turnValue), (power * sin + turnValue));
        return new Vector2D(velocity.getX(), velocity.getY());
    }


    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(-flPow);
        fr.setPower(frPow);
        bl.setPower(-blPow);
        br.setPower(brPow);
    }





}
/**
 * drive encoder constructor
 */
