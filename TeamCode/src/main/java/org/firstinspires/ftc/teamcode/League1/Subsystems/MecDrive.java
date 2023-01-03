package org.firstinspires.ftc.teamcode.League1.Subsystems;

////import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.League1.Common.OpModeWrapper;
import org.firstinspires.ftc.teamcode.League1.Common.Point;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Common.Utils;
import org.firstinspires.ftc.teamcode.League1.Common.Vector2D;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

////@Config
public class MecDrive {

    private DcMotorEx fl, fr, bl, br;
    BNO055IMU imu;
    private Robot robot;
    private boolean isDriveOnChub = true;
    private boolean pidEnabled;
    Telemetry telemetry;
    ColorRangeSensor color;
    double baseRed, baseBlue;


    //Original
    PIDCoefficients pidf = new PIDCoefficients(0.033, 0,0.00055);
    //PIDCoefficients pidf = new PIDCoefficients(0.031, 0,0.00055);

    //PIDCoefficients rotate = new PIDCoefficients(1.09, 0, 0.002);
    PIDCoefficients rotate = new PIDCoefficients(0.975, 0, 0.02);


    //PIDCoefficients rotateFaster = new PIDCoefficients(1.09, 0, 0.002);
    PIDCoefficients rotateFaster = new PIDCoefficients(0.975, 0, 0.02);



    //Slow start
    /*PIDFCoefficients pidf = new PIDFCoefficients(0.00075, 0.00042,0.0002, 0);


    PIDCoefficients rotate = new PIDCoefficients(0.75, 0.00015, 0.2);
    PIDCoefficients rotateFaster = new PIDCoefficients(0.85, 0.0002, 0.22);*/

    //Faster Slower Start
    /*PIDFCoefficients pidf = new PIDFCoefficients(0.0008, 0.00042,0.0002, 0);


     */
    //PIDCoefficients rotate = new PIDCoefficients(0.82, 0.00016, 0.2);
    //PIDCoefficients rotateFaster = new PIDCoefficients(0.92, 0.00021, 0.22);






    File loggingFile = AppUtil.getInstance().getSettingsFile("telemetry.txt");
    String loggingString;

    public enum MovementType{
        STRAIGHT,
        STRAFE,
        ROTATE,
        LDIAGONAL,
        RDIAGONAL,
        RDIAGONALLESS,
        LDIAGONALLESS
    }

    public enum DiagonalPath{
        REDRIGHT,
        REDLEFT,
        BLUERIGHT,
        BLUELEFT
    }

    /*
    localizer -> Arm/actuator -> drive
     */

    public MecDrive(HardwareMap hardwareMap, Robot robot , boolean pidEnabled, Telemetry telemetry){
        fl = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        br = hardwareMap.get(DcMotorEx.class, "BackRightDrive");
        this.robot = robot;
        this.pidEnabled = pidEnabled;
        this.telemetry = telemetry;

        robot.setShouldUpdate(false);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setShouldUpdate(true);







        CorrectMotors();
    }

    public MecDrive(HardwareMap hardwareMap, boolean pidEnabled, Telemetry telemetry){
        fl = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        br = hardwareMap.get(DcMotorEx.class, "BackRightDrive");
        this.pidEnabled = pidEnabled;
        this.telemetry = telemetry;

        //robot.setShouldUpdate(false);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.setShouldUpdate(true);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);





        CorrectMotors();
    }

    public MecDrive(HardwareMap hardwareMap, boolean pidEnabled, Telemetry telemetry, boolean usingImu){
        fl = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        br = hardwareMap.get(DcMotorEx.class, "BackRightDrive");
        this.pidEnabled = pidEnabled;
        this.telemetry = telemetry;

        //robot.setShouldUpdate(false);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.setShouldUpdate(true);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        CorrectMotors();
    }

    public MecDrive(HardwareMap hardwareMap, boolean pidEnabled, Telemetry telemetry, ColorRangeSensor color){
        fl = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        br = hardwareMap.get(DcMotorEx.class, "BackRightDrive");
        this.pidEnabled = pidEnabled;
        this.telemetry = telemetry;

        //robot.setShouldUpdate(false);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.setShouldUpdate(true);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        this.color = color;
        this.color.setGain(200);

        baseBlue = 0.6;
        baseRed = 0.4;




        CorrectMotors();
    }



    public int getFLEncoder(){
        return fl.getCurrentPosition();
    }

    public int getFREncoder(){
        return fr.getCurrentPosition();
    }

    public int getBLEncoder(){
        return bl.getCurrentPosition();
    }

    public int getBREncoder(){
        return br.getCurrentPosition();
    }


    public void rotate(double angle, double power){
        if(angle < 0){
            power *= -1;
        }

        angle = Utils.wrapAngle(angle);
        if(pidEnabled){
            setPIDRotateVelocity(angle, power);

        }else {

            while (Math.abs(robot.getDirection()) < Math.abs(angle)) {


                setPower(-power, power, -power, power);

            }
        }

        brake();

    }

    public void tankRotatePID(double radians, double power, boolean slidesUp){

        /*if(radians > imu.getAngularOrientation().firstAngle){
            power *= -1;
        }*/

        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        double actualStartTime = startTime;

        radians = wrapAngle(radians);
        double radError = wrapAngle(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - radians);
        double previousError = radError;
        double integralSum = 0;


        while(Math.abs(radError) > 0.005 && (time.seconds() - actualStartTime) < 1){

            telemetry.addData("target", radians);

            double newPower = 0;

            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

            double currentTime = time.seconds();

            radError = wrapAngle(currentAngle - radians);
            telemetry.addData("current", currentAngle);

            telemetry.addData("Error", radError);

            integralSum += (radError + previousError)/(currentTime - startTime);
            telemetry.addData("Integral", integralSum);

            //TODO:See if we need an integral limit
            if(integralSum > 10000){
                integralSum = 10000;
            }else if(integralSum < -10000){
                integralSum = -10000;
            }

            double derivative = (radError - previousError)/(currentTime - startTime);
            telemetry.addData("Derivative", derivative);


            //TODO: see if we should multiply by power at the end


            if(!slidesUp) {
                newPower = ((rotate.p * radError) + (rotate.i * integralSum) + (rotate.d * derivative));
            }else{
                newPower = ((rotateFaster.p * radError) + (rotateFaster.i * integralSum) + (rotateFaster.d * derivative));
            }
            setPowerAuto(newPower, MecDrive.MovementType.ROTATE);

            telemetry.addData("Power", newPower);

            startTime = currentTime;
            previousError = radError;
            telemetry.update();

        }

        simpleBrake();




    }



    public void tankRotate(double radians, double power){
        radians = wrapAngle(radians);


        if(radians > imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle){
            power *= -1;
        }




        while(Math.abs(radians - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle) > 0.01){

           setPowerAuto(power, MovementType.ROTATE);
        }

        simpleBrake();




    }


    public void tankRotatePID(double radians, double power, boolean slidesUp, double kickout){

        /*if(radians > imu.getAngularOrientation().firstAngle){
            power *= -1;
        }*/

        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        double actualStartTime = startTime;

        radians = wrapAngle(radians);
        double radError = wrapAngle(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - radians);
        double previousError = radError;
        double integralSum = 0;


        while(Math.abs(radError) > 0.005 && (time.seconds() - actualStartTime) < kickout){

            telemetry.addData("target", radians);

            double newPower = 0;

            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

            double currentTime = time.seconds();

            radError = wrapAngle(currentAngle - radians);
            telemetry.addData("current", currentAngle);

            telemetry.addData("Error", radError);

            integralSum += (radError + previousError)/(currentTime - startTime);
            telemetry.addData("Integral", integralSum);

            //TODO:See if we need an integral limit
            if(integralSum > 10000){
                integralSum = 10000;
            }else if(integralSum < -10000){
                integralSum = -10000;
            }

            double derivative = (radError - previousError)/(currentTime - startTime);
            telemetry.addData("Derivative", derivative);


            //TODO: see if we should multiply by power at the end


            if(!slidesUp) {
                newPower = ((rotate.p * radError) + (rotate.i * integralSum) + (rotate.d * derivative));
            }else{
                newPower = ((rotateFaster.p * radError) + (rotateFaster.i * integralSum) + (rotateFaster.d * derivative));
            }
            setPowerAuto(newPower, MecDrive.MovementType.ROTATE);

            telemetry.addData("Power", newPower);

            startTime = currentTime;
            previousError = radError;
            telemetry.update();

        }

        simpleBrake();




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







    public void newMoveToPosition(Point p, double power) {
        double direction = Utils.wrapAngle(p.getDirection()) - (Math.PI / 4);

        int flPosition = Math.abs((int) (Math.cos(direction) * p.magFromOrigin()));
        int frPosition = Math.abs((int) (Math.sin(direction) * p.magFromOrigin()));
        int blPosition = Math.abs((int) (Math.sin(direction) * p.magFromOrigin()));
        int brPosition = Math.abs((int) (Math.cos(direction) * p.magFromOrigin()));

        /*double flVelocity = Math.max(1, velocity * Math.cos(direction));
        double frVelocity = Math.max(1, velocity * Math.sin(direction));
        double blVelocity = Math.max(1, velocity * Math.sin(direction));
        double brVelocity = Math.max(1, velocity * Math.cos(direction));
*/
        double flPower = Math.cos(direction) * power;
        double frPower = Math.sin(direction) * power;
        double blPower = Math.sin(direction) * power;
        double brPower = Math.cos(direction) * power;


        if (pidEnabled) {
            //setPIDVelocity(flVelocity, flPosition, frVelocity, frPosition, blVelocity, blPosition, brVelocity, brPosition);
        } else {


            LynxModule.BulkData data = robot.getBulkPacket(isDriveOnChub);

            int flPos = data.getMotorCurrentPosition(0);
            int frPos = data.getMotorCurrentPosition(1);
            int blPos = data.getMotorCurrentPosition(2);
            int brPos = data.getMotorCurrentPosition(3);

            //TODO: might need to change this direction thing
            if(direction + (Math.PI/4) > 0) {
                frPos *= -1;
                brPos *= -1;
            }else{
                flPos *= -1;
                blPos *= -1;
            }
            //TODO:check if might need to change abs condition
            while (opModeIsRunning() && (flPos < flPosition || frPos < frPosition || blPos < blPosition || brPos < brPosition)) {
                //setMotorVelocity(flVelocity, frVelocity, blVelocity, brVelocity);

                if (flPos >= flPosition) {
                    flPower = 0;

                } else if (frPos >= frPosition) {
                    frPower = 0;

                } else if (blPos >= blPosition) {
                    blPower = 0;

                } else if (brPos >= brPosition) {
                    brPower = 0;

                }

                setPower(flPower, frPower, blPower, brPower);


                //setPower(0.3, 0.3, 0.3, 0.3);
                data = robot.getBulkPacket(isDriveOnChub);

                flPos = data.getMotorCurrentPosition(0);
                frPos = data.getMotorCurrentPosition(1);
                blPos = data.getMotorCurrentPosition(2);
                brPos = data.getMotorCurrentPosition(3);

                if(direction + (Math.PI/4) > 0) {
                    frPos *= -1;
                    brPos *= -1;
                }else{
                    flPos *= -1;
                    blPos *= -1;
                }

                //Telemetry and logging for current motor positions
                telemetry.addData("fl: ", flPos);
                telemetry.addData("fr: ", frPos);
                telemetry.addData("bl: ", blPos);
                telemetry.addData("br: ", brPos);

                loggingString += "flCurrentPosition: " + flPos + "\n";
                loggingString += "frCurrentPosition: " + flPos + "\n";
                loggingString += "blCurrentPosition: " + flPos + "\n";
                loggingString += "brCurrentPosition: " + flPos + "\n";
                loggingString += "\n";

                //Telemetry and logging for target motor positions
                telemetry.addData("target fl: ", flPosition);
                telemetry.addData("target fr: ", frPosition);
                telemetry.addData("target bl: ", blPosition);
                telemetry.addData("target br: ", brPosition);

                loggingString += "flTargetPosition: " + flPosition + "\n";
                loggingString += "frTargetPosition: " + frPosition + "\n";
                loggingString += "blTargetPosition: " + blPosition + "\n";
                loggingString += "brTargetPosition: " + brPosition + "\n";
                loggingString += "\n";

                //Telemetry and logging for power that each motor is set to
                telemetry.addData("flPower: ", flPower);
                telemetry.addData("frPower: ", frPower);
                telemetry.addData("blPower: ", blPower);
                telemetry.addData("brPower: ", brPower);

                loggingString += "flPower: " + flPower + "\n";
                loggingString += "frPower: " + frPower + "\n";
                loggingString += "blPower: " + blPower + "\n";
                loggingString += "brPower: " + brPower + "\n";
                loggingString += "\n";
                loggingString += "\n";
                loggingString += "\n";



                telemetry.update();

            }
            brake();
        }
    }

    public double getFirstAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    //TODO: Test this
    public void simpleMoveToPosition(int tics, MovementType movement, double power){

        if(avgPos() > tics){
            power *= -1;
        }
        while(avgPos() < Math.abs(tics)){
            setPowerAuto(power, movement);
        }

        simpleBrake();

    }

    public double avgPos(){
        return (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition())
                + Math.abs(bl.getCurrentPosition()) /*+ Math.abs(br.getCurrentPosition()))*/) / 4;
    }




    public void moveToPosition(Point p, double velocity){

        double robotDirection = robot.getDirection();
        double direction = Utils.wrapAngle(p.getDirection()) - Utils.wrapAngle(robotDirection);

        int flPosition = (int)(Math.sin(direction) * p.magFromOrigin()); //Direction Math.Pi/4 -> 0
        int frPosition = (int)(Math.cos(direction) * p.magFromOrigin());
        int blPosition = (int)(Math.cos(direction) * p.magFromOrigin());
        int brPosition = (int)(Math.sin(direction) * p.magFromOrigin());

        double flVelocity = Math.max(1, velocity * Math.sin(direction));
        double frVelocity = Math.max(1, velocity * Math.cos(direction));
        double blVelocity = Math.max(1, velocity * Math.cos(direction));
        double brVelocity = Math.max(1, velocity * Math.sin(direction));


/*

        double flPower = Math.sin(direction) * 0.5;
        double frPower = Math.cos(direction) * 0.5;
        double blPower = Math.cos(direction) * 0.5;
        double brPower = Math.sin(direction) * 0.5;
*/



        if(pidEnabled){
            setPIDVelocity(flVelocity, flPosition, frVelocity, frPosition, blVelocity, blPosition, brVelocity, brPosition);
        }else {
            LynxModule.BulkData data = robot.getBulkPacket(isDriveOnChub);

            int flPos = data.getMotorCurrentPosition(0);
            int frPos = data.getMotorCurrentPosition(1);
            int blPos = data.getMotorCurrentPosition(2);
            int brPos = data.getMotorCurrentPosition(3);
            while (opModeIsRunning() && flPos < flPosition && frPos < frPosition && blPos < blPosition && brPos < brPosition) {
                //setMotorVelocity(flVelocity, frVelocity, blVelocity, brVelocity);

                setPower(0.3, 0.3, 0.3, 0.3);
                data = robot.getBulkPacket(isDriveOnChub);

                flPos = data.getMotorCurrentPosition(0);
                frPos = data.getMotorCurrentPosition(1);
                blPos = data.getMotorCurrentPosition(2);
                brPos = data.getMotorCurrentPosition(3);

                telemetry.addData("fl: ", flPos);
                telemetry.addData("fr: ", frPos);
                telemetry.addData("bl: ", blPos);
                telemetry.addData("br: ", brPos);

                telemetry.addData("target fl: ", flPosition);
                telemetry.addData("target fr: ", frPosition);
                telemetry.addData("target bl: ", blPosition);
                telemetry.addData("target br: ", brPosition);


                telemetry.update();

            }
            brake();
        }
    }

    public void simpleBrake() {
        setPower(0,0,0,0);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    private void brake() {
        robot.setShouldUpdate(false);
        setPower(0,0,0,0);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setShouldUpdate(true);

    }

    private void setMotorVelocity(double flV, double frV, double blV, double brV){
        fl.setVelocity(flV);
        fr.setVelocity(frV);
        bl.setVelocity(blV);
        br.setVelocity(brV);
    }

    private void setMotorPower(double flP, double frP, double blP, double brP){
        fl.setPower(flP);
        fr.setPower(frP);
        bl.setPower(blP);
        br.setPower(brP);
    }


    private double P = 0.1; //TODO starting values to be tuned with Zeiger Nichols method
    private double I = 0;
    private double D = 0;
    private double F = 0;

    double integralSumLimit = 1000;

    /**
     *
     * @param flV target velocity d/dx position
     * @param flPosition target
     * Zeiger-Nichols method
     * low pass filtering
     */
    private void setPIDVelocity(double flV, int flPosition, double frV, int frPosition, double blV, int blPosition, double brV, int brPosition){
        double flIntegralSum = 0;
        double flPreviousError = 0;
        double frIntegralSum = 0;
        double frPreviousError = 0;
        double blIntegralSum = 0;
        double blPreviousError = 0;
        double brIntegralSum = 0;
        double brPreviousError = 0;

        double start = OpModeWrapper.currentOpMode().time; //start time
        LynxModule.BulkData data = robot.getBulkPacket(isDriveOnChub);
        int flCurr = data.getMotorCurrentPosition(0); //init motor pos'
        int frCurr = data.getMotorCurrentPosition(1);
        int blCurr = data.getMotorCurrentPosition(2);
        int brCurr = data.getMotorCurrentPosition(3);

        //low pass fields
//        double tau = 0.8;
//        double previousFilterEstimate = 0;
//        double currentFilterEstimate = 0;

        while(opModeIsRunning() && flCurr < flPosition && frCurr < frPosition && blCurr < blPosition && brCurr < brPosition){

            data = robot.getBulkPacket(isDriveOnChub);
            flCurr = data.getMotorCurrentPosition(0);
            frCurr = data.getMotorCurrentPosition(1);
            blCurr = data.getMotorCurrentPosition(2);
            brCurr = data.getMotorCurrentPosition(3);


            double flError = flPosition - flCurr; //error, P
            double frError = frPosition - frCurr;
            double blError = blPosition - blCurr;
            double brError = brPosition - brCurr;

            //low pass impl, note to write this for ALL motors if implemented, keep Tau constant among all 4 wheels/sensors
            //currentFilterEstimate = (tau * previousFilterEstimate) + (1-tau) * flError - previousError;
            //double flDerivative = currentFilterEstimate / elapsedTime;

            double elapsedTime = OpModeWrapper.currentOpMode().time - start; //elapsed time since start of the movement

            double flDerivative = (flError - flPreviousError) / elapsedTime; //Rate of Change, Error/time
            double frDerivative = (frError - frPreviousError) / elapsedTime;
            double blDerivative = (blError - blPreviousError) / elapsedTime;
            double brDerivative = (brError - brPreviousError) / elapsedTime;


            flIntegralSum = flIntegralSum + (flCurr * elapsedTime); //sum of error
            frIntegralSum = frIntegralSum + (frCurr * elapsedTime);
            blIntegralSum = blIntegralSum + (blCurr * elapsedTime);
            brIntegralSum = brIntegralSum + (brCurr * elapsedTime);

            if(flIntegralSum > integralSumLimit){
                flIntegralSum = integralSumLimit;
            }else if (flIntegralSum < - integralSumLimit){
                flIntegralSum = -integralSumLimit;
            }
            if(frIntegralSum > integralSumLimit){
                frIntegralSum = integralSumLimit;
            }else if (frIntegralSum < - integralSumLimit){
                frIntegralSum = -integralSumLimit;
            }
            if(blIntegralSum > integralSumLimit){
                blIntegralSum = integralSumLimit;
            }else if (blIntegralSum < - integralSumLimit){
                blIntegralSum = -integralSumLimit;
            }
            if(brIntegralSum > integralSumLimit){
                brIntegralSum = integralSumLimit;
            }else if (brIntegralSum < - integralSumLimit){
                brIntegralSum = -integralSumLimit;
            }


            double flOut = (P * flError) + (I * flIntegralSum) + (D * flDerivative) + F;
            double frOut = (P * frError) + (I * frIntegralSum) + (D * frDerivative) + F;
            double blOut = (P * blError) + (I * blIntegralSum) + (D * blDerivative) + F;
            double brOut = (P * brError) + (I * brIntegralSum) + (D * brDerivative) + F;

            setMotorVelocity(flOut * flV, frOut * frV,blOut * blV,brOut * brV);

            flPreviousError = flError;
            frPreviousError = frError;
            blPreviousError = blError;
            brPreviousError = brError;
            //previousFilterEstimate = currentFilterEstimate;
        }
        brake();
    }

    public void goTOPIDPos(int tics, double power, MovementType movement){
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        double actualStartTime = startTime;
        //boolean countTime = true;

        boolean isInitialErrorNegative;

        int half = Math.abs(tics) / 2;



        //TODO: check if we need to negate any
        int flPos = -1 * getFLEncoder();
        int frPos = getFREncoder();
        int blPos = -1 * getBLEncoder();
        int brPos = getBREncoder();

        int flError = tics - flPos;
        int frError = tics - frPos;
        int blError = tics - blPos;
        int brError = tics - brPos;

        int flPreviousError = flError;
        int frPreviousError = frError;
        int blPreviousError = blError;
        int brPreviousError = brError;

        if(flPreviousError < 0){
            isInitialErrorNegative = true;
        }else{
            isInitialErrorNegative = false;
        }

        int flIntegralSum = 0;
        int frIntegralSum = 0;
        int blIntegralSum = 0;
        int brIntegralSum = 0;


        while(Math.abs(flError) > 2 && Math.abs(frError) > 2 && Math.abs(blError) > 2 && Math.abs(brError) > 2 && (time.seconds() - actualStartTime) < 1.5){
            telemetry.addData("target", tics);


            //TODO: check if we need to negate any
            flPos = -1 * getFLEncoder();
            frPos = getFREncoder();
            blPos = -1 * getBLEncoder();
            brPos = getBREncoder();



            telemetry.addData("flPos", flPos);
            telemetry.addData("frPos", frPos);
            telemetry.addData("blPos", blPos);
            telemetry.addData("brPos", brPos);

            flError = tics - flPos;
            frError = tics - frPos;
            blError = tics - blPos;
            brError = tics - brPos;



            double currentTime = time.seconds();

            telemetry.addData("flError", flError);
            telemetry.addData("frError", frError);
            telemetry.addData("blError", blError);
            telemetry.addData("brError", brError);

            flIntegralSum += (0.5 * (flError + flPreviousError) * (currentTime - startTime));
            frIntegralSum += (0.5 * (frError + frPreviousError) * (currentTime - startTime));
            blIntegralSum += (0.5 * (blError + blPreviousError) * (currentTime - startTime));
            brIntegralSum += (0.5 * (brError + brPreviousError) * (currentTime - startTime));

            telemetry.addData("flIntegralSum", flIntegralSum);
            telemetry.addData("frIntegralSum", frIntegralSum);
            telemetry.addData("blIntegralSum", blIntegralSum);
            telemetry.addData("brIntegralSum", brIntegralSum);


            //TODO: look at telemetry and see if we can have new bound (change integral sum limit)
            if(flIntegralSum > 20000){
                flIntegralSum = 20000;
            }else if(flIntegralSum < -20000){
                flIntegralSum = -20000;
            }

            if(frIntegralSum > 20000){
                frIntegralSum = 20000;
            }else if(frIntegralSum < -20000){
                frIntegralSum = -20000;
            }

            if(blIntegralSum > 20000){
                blIntegralSum = 20000;
            }else if(blIntegralSum < -20000){
                blIntegralSum = -20000;
            }

            if(brIntegralSum > 20000){
                brIntegralSum = 20000;
            }else if(brIntegralSum < -20000){
                brIntegralSum = -20000;
            }


            double flDerivative = (flError - flPreviousError)/(currentTime - startTime);
            double frDerivative = (frError - frPreviousError)/(currentTime - startTime);
            double blDerivative = (blError - blPreviousError)/(currentTime - startTime);
            double brDerivative = (brError - brPreviousError)/(currentTime - startTime);


            //TODO: see if we need this
            /*if(!(Math.abs(flError) < half && Math.abs(frError) < half && Math.abs(blError) < half && Math.abs(brError) < half)){
                flDerivative = 0;
                frDerivative = 0;
                blDerivative = 0;
                brDerivative = 0;
            }
*/
            /*//TODO: Max out integral if shoot past
            if(Math.abs(flError) > 10 || Math.abs(frError) > 10 || Math.abs(blError) > 10 || Math.abs(brError) > 10){
                if(isInitialErrorNegative){
                    if(flError > 0){
                        flIntegralSum = 500;
                    }

                    if(frError > 0){
                        frIntegralSum = 500;
                    }

                    if(blIntegralSum > 0){
                        blIntegralSum = 500;
                    }

                    if(brIntegralSum > 0){
                        brIntegralSum = 500;

                    }
                }else{
                    if(flError < 0){
                        flIntegralSum = -500;
                    }

                    if(frError < 0){
                        frIntegralSum = -500;
                    }

                    if(blIntegralSum < 0){
                        blIntegralSum = -500;
                    }

                    if(brIntegralSum < 0){
                        brIntegralSum = -500;

                    }

                }
                *//*if(flError > 0 && frError > 0 && blError > 0 && brError > 0 && isInitialErrorNegative){
                    flIntegralSum = 500;
                    frIntegralSum = 500;
                    blIntegralSum = 500;
                    brIntegralSum = 500;

                }else if(flError < 0 && frError < 0 && blError < 0 && brError < 0 && !isInitialErrorNegative){
                    flIntegralSum = -500;
                    frIntegralSum = -500;
                    blIntegralSum = -500;
                    brIntegralSum = -500;
                }*//*

            }

*/


            telemetry.addData("flDerivative", flDerivative);
            telemetry.addData("frDerivative", frDerivative);
            telemetry.addData("blDerivative", blDerivative);
            telemetry.addData("brDerivative", brDerivative);

            double flPower = ((pidf.p * flError) + (pidf.i * flIntegralSum) + (pidf.d * flDerivative));
            double frPower = ((pidf.p * frError) + (pidf.i * frIntegralSum) + (pidf.d * frDerivative));
            double blPower = ((pidf.p * blError) + (pidf.i * blIntegralSum) + (pidf.d * blDerivative));
            double brPower = ((pidf.p * brError) + (pidf.i * brIntegralSum) + (pidf.d * brDerivative));


            //TODO: See if we need this maxing out for power

            /*if(Math.abs(flPower) > Math.abs(power)){
                if(flPower < 0){
                    flPower = -power;
                }else{
                    flPower = power;
                }
            }

            if(Math.abs(frPower) > Math.abs(power)){
                if(frPower < 0){
                    frPower = -power;
                }else{
                    frPower = power;
                }
            }

            if(Math.abs(blPower) > Math.abs(power)){
                if(blPower < 0){
                    blPower = -power;
                }else{
                    blPower = power;
                }
            }

            if(Math.abs(brPower) > Math.abs(power)){
                if(brPower < 0){
                    brPower = -power;
                }else{
                    brPower = power;
                }
            }
*/

            telemetry.addData("flPower", flPower);
            telemetry.addData("frPower", frPower);
            telemetry.addData("blPower", blPower);
            telemetry.addData("brPower", brPower);

            //TODO: check if we need setting power to negative (I dont think we need it)
            /*if(tics < flPos){
                flPower *= -1;
            }

            if(tics < frPos){
                flPower *= -1;
            }

            if(tics < frPos){
                flPower *= -1;
            }

            if(tics < frPos){
                flPower *= -1;
            }
*/
            //TODO: Fix rotate and check Strafe
            if(movement == MovementType.STRAIGHT) {
                setPower(flPower, frPower, blPower, brPower);
            }else if(movement == MovementType.ROTATE){
                setPower(flPower, -frPower, blPower, -brPower);
            }else if(movement == MovementType.STRAFE){
                setPower(flPower, -frPower, -blPower, brPower);
            }


            startTime = currentTime;
            flPreviousError = flError;
            frPreviousError = frError;
            blPreviousError = blError;
            brPreviousError = brError;

            telemetry.update();


        }

        //setPower(0,0,0,0);
        simpleBrake();

    }


    public void goTOPIDPos(int tics, double power, MovementType movement, double kickout) {
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        double actualStartTime = startTime;
        //boolean countTime = true;

        boolean isInitialErrorNegative;

        int half = Math.abs(tics) / 2;



        //TODO: check if we need to negate any
        int flPos = -1 * getFLEncoder();
        int frPos = getFREncoder();
        int blPos = -1 * getBLEncoder();
        int brPos = getBREncoder();

        int flError = tics - flPos;
        int frError = tics - frPos;
        int blError = tics - blPos;
        int brError = tics - brPos;

        int flPreviousError = flError;
        int frPreviousError = frError;
        int blPreviousError = blError;
        int brPreviousError = brError;

        if(flPreviousError < 0){
            isInitialErrorNegative = true;
        }else{
            isInitialErrorNegative = false;
        }

        int flIntegralSum = 0;
        int frIntegralSum = 0;
        int blIntegralSum = 0;
        int brIntegralSum = 0;


        while(Math.abs(flError) > 2 && Math.abs(frError) > 2 && Math.abs(blError) > 2 && Math.abs(brError) > 2 && (time.seconds() - actualStartTime) < kickout){
            telemetry.addData("target", tics);


            //TODO: check if we need to negate any
            flPos = -1 * getFLEncoder();
            frPos = getFREncoder();
            blPos = -1 * getBLEncoder();
            brPos = getBREncoder();



            telemetry.addData("flPos", flPos);
            telemetry.addData("frPos", frPos);
            telemetry.addData("blPos", blPos);
            telemetry.addData("brPos", brPos);

            flError = tics - flPos;
            frError = tics - frPos;
            blError = tics - blPos;
            brError = tics - brPos;



            double currentTime = time.seconds();

            telemetry.addData("flError", flError);
            telemetry.addData("frError", frError);
            telemetry.addData("blError", blError);
            telemetry.addData("brError", brError);

            flIntegralSum += (0.5 * (flError + flPreviousError) * (currentTime - startTime));
            frIntegralSum += (0.5 * (frError + frPreviousError) * (currentTime - startTime));
            blIntegralSum += (0.5 * (blError + blPreviousError) * (currentTime - startTime));
            brIntegralSum += (0.5 * (brError + brPreviousError) * (currentTime - startTime));

            telemetry.addData("flIntegralSum", flIntegralSum);
            telemetry.addData("frIntegralSum", frIntegralSum);
            telemetry.addData("blIntegralSum", blIntegralSum);
            telemetry.addData("brIntegralSum", brIntegralSum);


            //TODO: look at telemetry and see if we can have new bound (change integral sum limit)
            if(flIntegralSum > 20000){
                flIntegralSum = 20000;
            }else if(flIntegralSum < -20000){
                flIntegralSum = -20000;
            }

            if(frIntegralSum > 20000){
                frIntegralSum = 20000;
            }else if(frIntegralSum < -20000){
                frIntegralSum = -20000;
            }

            if(blIntegralSum > 20000){
                blIntegralSum = 20000;
            }else if(blIntegralSum < -20000){
                blIntegralSum = -20000;
            }

            if(brIntegralSum > 20000){
                brIntegralSum = 20000;
            }else if(brIntegralSum < -20000){
                brIntegralSum = -20000;
            }


            double flDerivative = (flError - flPreviousError)/(currentTime - startTime);
            double frDerivative = (frError - frPreviousError)/(currentTime - startTime);
            double blDerivative = (blError - blPreviousError)/(currentTime - startTime);
            double brDerivative = (brError - brPreviousError)/(currentTime - startTime);

            telemetry.addData("flDerivative", flDerivative);
            telemetry.addData("frDerivative", frDerivative);
            telemetry.addData("blDerivative", blDerivative);
            telemetry.addData("brDerivative", brDerivative);

            double flPower = ((pidf.p * flError) + (pidf.i * flIntegralSum) + (pidf.d * flDerivative));
            double frPower = ((pidf.p * frError) + (pidf.i * frIntegralSum) + (pidf.d * frDerivative));
            double blPower = ((pidf.p * blError) + (pidf.i * blIntegralSum) + (pidf.d * blDerivative));
            double brPower = ((pidf.p * brError) + (pidf.i * brIntegralSum) + (pidf.d * brDerivative));

            //TODO: See if we need this maxing out for power

            telemetry.addData("flPower", flPower);
            telemetry.addData("frPower", frPower);
            telemetry.addData("blPower", blPower);
            telemetry.addData("brPower", brPower);

            //TODO: Fix rotate and check Strafe
            if(movement == MovementType.STRAIGHT) {
                setPower(flPower, frPower, blPower, brPower);
            }else if(movement == MovementType.ROTATE){
                setPower(flPower, -frPower, blPower, -brPower);
            }else if(movement == MovementType.STRAFE){
                setPower(flPower, -frPower, -blPower, brPower);
            }


            startTime = currentTime;
            flPreviousError = flError;
            frPreviousError = frError;
            blPreviousError = blError;
            brPreviousError = brError;

            telemetry.update();


        }

        //setPower(0,0,0,0);
        simpleBrake();

    }


    public int avgPosPID(){
        return ((-1 * fl.getCurrentPosition()) + fr.getCurrentPosition()
                + (-1 * bl.getCurrentPosition()) + br.getCurrentPosition()) / 4;
    }

    /*
    public void goTOPIDPosAvg(int tics, double power, MovementType movement){
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();



        //TODO: check if we need to negate any
        int avgPos = avgPosPID();

        int error = tics - avgPos;

        int previousError = avgPos;

        int integralSum = 0;

        while(Math.abs(error) > 2){
            telemetry.addData("target", tics);

            //TODO: check if we need to negate any
            avgPos = avgPosPID();


            telemetry.addData("avgPos", avgPos);
            telemetry.addData("flPos", getFLEncoder());
            telemetry.addData("frPos", getFREncoder());
            telemetry.addData("blPos", getBLEncoder());
            telemetry.addData("brPos", getBREncoder());


            error = tics - avgPos;


            double currentTime = time.seconds();

            telemetry.addData("error", error);


            integralSum += (0.5 * (error + previousError) * (currentTime - startTime));

            telemetry.addData("integralSum", integralSum);



            //TODO: look at telemetry and see if we can have new bound (change integral sum limit)
            if(integralSum > 20000){
                integralSum = 20000;
            }else if(integralSum < -20000){
                integralSum = -20000;
            }


            double derivative = (error - previousError)/(currentTime - startTime);


            telemetry.addData("derivative", derivative);


            double finalPower = (pidf.p * error) + (pidf.i * integralSum) + (pidf.d * derivative) + (pidf.f * power);



            //TODO: See if we need this maxing out for power

            /*if(Math.abs(flPower) > Math.abs(power)){
                if(flPower < 0){
                    flPower = -power;
                }else{
                    flPower = power;
                }
            }

            if(Math.abs(frPower) > Math.abs(power)){
                if(frPower < 0){
                    frPower = -power;
                }else{
                    frPower = power;
                }
            }

            if(Math.abs(blPower) > Math.abs(power)){
                if(blPower < 0){
                    blPower = -power;
                }else{
                    blPower = power;
                }
            }

            if(Math.abs(brPower) > Math.abs(power)){
                if(brPower < 0){
                    brPower = -power;
                }else{
                    brPower = power;
                }
            }



            //TODO: check if we need setting power to negative (I dont think we need it)
            if(tics < flPos){
                flPower *= -1;
            }

            if(tics < frPos){
                flPower *= -1;
            }

            if(tics < frPos){
                flPower *= -1;
            }

            if(tics < frPos){
                flPower *= -1;
            }


            if(movement == MovementType.STRAIGHT) {
                setPower(power, power, power, power);
            }/*else if(movement == MovementType.ROTATE){
                setPower(flPower, -frPower, blPower, -brPower);
            }else if(movement == MovementType.STRAFE){
                setPower(flPower, -frPower, -blPower, brPower);
            }


            startTime = currentTime;
            previousError = error;
            telemetry.update();



        }

        //setPower(0,0,0,0);
        simpleBrake();

    }
    */





    public void PIDPowerNoBulk(double power, int targetPosition){
        double flIntegralSum = 0;
        double flPreviousError = 0;
        double frIntegralSum = 0;
        double frPreviousError = 0;
        double blIntegralSum = 0;
        double blPreviousError = 0;
        double brIntegralSum = 0;
        double brPreviousError = 0;

        double start = OpModeWrapper.currentOpMode().time; //start time

        //TODO: may need to negate some things
        int flCurr = fl.getCurrentPosition(); //init motor pos'
        int frCurr = fr.getCurrentPosition();
        int blCurr = bl.getCurrentPosition();
        int brCurr = br.getCurrentPosition();


        //low pass fields
//        double tau = 0.8;
//        double previousFilterEstimate = 0;
//        double currentFilterEstimate = 0;

        //TODO: change this logic so that forwards and backwards works along with strafing
        while(opModeIsRunning() && flCurr < targetPosition && frCurr < targetPosition && blCurr < targetPosition && brCurr < targetPosition){

            telemetry.addData("fl", fl.getCurrentPosition());
            telemetry.addData("fr", fr.getCurrentPosition());
            telemetry.addData("bl", bl.getCurrentPosition());
            telemetry.addData("br", br.getCurrentPosition());


            //TODO: may need to negate some things
            flCurr = fl.getCurrentPosition();
            frCurr = fr.getCurrentPosition();
            blCurr = bl.getCurrentPosition();
            brCurr = br.getCurrentPosition();

            double flError = targetPosition - flCurr; //error, P
            double frError = targetPosition - frCurr;
            double blError = targetPosition - blCurr;
            double brError = targetPosition - brCurr;

            telemetry.addData("flError", flError);
            telemetry.addData("frError", frError);
            telemetry.addData("blError", blError);
            telemetry.addData("brError", brError);


            //low pass impl, note to write this for ALL motors if implemented, keep Tau constant among all 4 wheels/sensors
            //currentFilterEstimate = (tau * previousFilterEstimate) + (1-tau) * flError - previousError;
            //double flDerivative = currentFilterEstimate / elapsedTime;

            double elapsedTime = OpModeWrapper.currentOpMode().time - start; //elapsed time since start of the movement

            double flDerivative = (flError - flPreviousError) / elapsedTime; //Rate of Change, Error/time
            double frDerivative = (frError - frPreviousError) / elapsedTime;
            double blDerivative = (blError - blPreviousError) / elapsedTime;
            double brDerivative = (brError - brPreviousError) / elapsedTime;

            telemetry.addData("flDerivative", flDerivative);
            telemetry.addData("frDerivative", frDerivative);
            telemetry.addData("blDerivative", blDerivative);
            telemetry.addData("brDerivative", brDerivative);


            flIntegralSum = flIntegralSum + (flCurr * elapsedTime); //sum of error
            frIntegralSum = frIntegralSum + (frCurr * elapsedTime);
            blIntegralSum = blIntegralSum + (blCurr * elapsedTime);
            brIntegralSum = brIntegralSum + (brCurr * elapsedTime);

            telemetry.addData("flIntegral", flIntegralSum);
            telemetry.addData("frIntegral", frIntegralSum);
            telemetry.addData("blIntegral", blIntegralSum);
            telemetry.addData("brIntegral", brIntegralSum);

            if(flIntegralSum > integralSumLimit){
                flIntegralSum = integralSumLimit;
            }else if (flIntegralSum < - integralSumLimit){
                flIntegralSum = -integralSumLimit;
            }
            if(frIntegralSum > integralSumLimit){
                frIntegralSum = integralSumLimit;
            }else if (frIntegralSum < - integralSumLimit){
                frIntegralSum = -integralSumLimit;
            }
            if(blIntegralSum > integralSumLimit){
                blIntegralSum = integralSumLimit;
            }else if (blIntegralSum < - integralSumLimit){
                blIntegralSum = -integralSumLimit;
            }
            if(brIntegralSum > integralSumLimit){
                brIntegralSum = integralSumLimit;
            }else if (brIntegralSum < - integralSumLimit){
                brIntegralSum = -integralSumLimit;
            }


            double flOut = (P * flError) + (I * flIntegralSum) + (D * flDerivative) + F;
            double frOut = (P * frError) + (I * frIntegralSum) + (D * frDerivative) + F;
            double blOut = (P * blError) + (I * blIntegralSum) + (D * blDerivative) + F;
            double brOut = (P * brError) + (I * brIntegralSum) + (D * brDerivative) + F;

            telemetry.addData("flOut", flOut);
            telemetry.addData("frOut", frOut);
            telemetry.addData("blOut", blOut);
            telemetry.addData("brOut", brOut);

            setMotorPower(flOut * power, frOut * power,blOut * power,brOut * power);

            flPreviousError = flError;
            frPreviousError = frError;
            blPreviousError = blError;
            brPreviousError = brError;
            //previousFilterEstimate = currentFilterEstimate;

            telemetry.update();
        }
        simpleBrake();
    }



    PIDCoefficients pid = new PIDCoefficients(0,0,0);

    //TODO: see if can make more efficient using timer.reset() and maybe add pid for each motor
    private void setPIDRotateVelocity(double targetAngle, double targetPower){
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        double integralSum = 0;

        //TODO: Check this line (check logic)
        double previousError = targetAngle;


        while(opModeIsRunning() && robot.getDirection() < targetAngle){
            double error = Utils.wrapAngle(targetAngle - robot.getDirection());
            double currentTime = time.seconds();

            //TODO: add an integral sum limit
            integralSum += (0.5 * (currentTime - startTime) * (previousError + error));


            double derivative = (error - previousError)/(currentTime - startTime);

            double out = (pid.p * error) + (pid.i * integralSum) + (pid.d * derivative);



            if(targetAngle < 0){
                out *= -1;
            }

            setPower(-out * targetPower, out * targetPower, -out * targetPower, out * targetPower);



            previousError = error;
            startTime = currentTime;

        }

        brake();


    }


    private boolean opModeIsRunning(){
        return OpModeWrapper.currentOpMode().opModeIsActive() && !OpModeWrapper.currentOpMode().isStopRequested();
    }
    /*
    46

     */

    private void CorrectMotors() {
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void coast() {
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    /*
    public synchronized void driveAuto(double desiredVelocity, int tics, MecanumDriveTrain.MovementType movement){

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LynxModule.BulkData data = robot.getBulkPacket(isDriveOnChub);


        OpModeWrapper currentOpMode = OpModeWrapper.currentOpMode();
        double startTime = currentOpMode.time;

        if(tics < 0) {
            desiredVelocity *= -1;
        }
        double elapsedTime = 0;

        while(isFar(tics) && currentOpMode.opModeIsActive() && elapsedTime < 5.0){
            elapsedTime = currentOpMode.time - startTime;
            setPowerAuto(desiredVelocity, movement);
            data = robot.getBulkPacket(isDriveOnChub);
            telemetry.addData("fl ", data.getMotorCurrentPosition(0));
            telemetry.addData("fr ", data.getMotorCurrentPosition(1));
            telemetry.addData("bl ", data.getMotorCurrentPosition(2));
            telemetry.addData("br ", data.getMotorCurrentPosition(3));
            telemetry.update();

        }

        brake();
    }

     */

    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(-flPow);
        fr.setPower(frPow) ;
        bl.setPower(-blPow);
        br.setPower(brPow);
    }


    public void setPower(Vector2D velocity, double turnValue, boolean isSwapped){
        turnValue = -turnValue;
        double direction =  velocity.getDirection();


        double power = velocity.magnitude();

        double angle = direction + 3*Math.PI / 4.0;
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        if(!isSwapped) {
            setPower((power * sin - turnValue), (power * cos + turnValue),
                    (power * cos - turnValue), (power * sin + turnValue));
        } else {
            setPower(-(power * sin - turnValue), -(power * cos + turnValue),
                    -(power * cos - turnValue), -(power * sin + turnValue));
        }
    }


    public double setPowerAuto(double power, MecDrive.MovementType movement) {
        if(movement == MecDrive.MovementType.STRAIGHT) {
            setPower(power, power, power, power);
        }else if(movement == MecDrive.MovementType.STRAFE){
            setPower(power, -power, -power, power);
        }else if(movement == MecDrive.MovementType.ROTATE){
            setPower(power, -power, power, -power);
        }else if(movement == MecDrive.MovementType.LDIAGONAL){
            setPower(0, power, power,0);
        }else if(movement == MecDrive.MovementType.RDIAGONAL){
            //setPower(0, power, power, 0);
            setPower(power, 0, 0, power);
        }else if(movement == MecDrive.MovementType.RDIAGONALLESS){
            //setPower(0, power, power, 0);
            setPower(power, power/1.35, power/1.35, power);
        }else if(movement == MecDrive.MovementType.LDIAGONALLESS){
            //setPower(0, power, power, 0);
            setPower(power/1.35, power, power, power/1.35);
        }
        return power;
    }


    //random comment
    private double TIC_TOLERANCE = 25;
    private boolean isFar(int tics){
        LynxModule.BulkData data = robot.getBulkPacket(isDriveOnChub);

        return Math.abs(tics - data.getMotorCurrentPosition(0)) > TIC_TOLERANCE && Math.abs(tics - data.getMotorCurrentPosition(1)) > TIC_TOLERANCE
                && Math.abs(tics - data.getMotorCurrentPosition(2)) > TIC_TOLERANCE && Math.abs(tics - data.getMotorCurrentPosition(3)) > TIC_TOLERANCE;
    }

    public void autoDiagonals(boolean startGoingLeft, boolean longer, boolean toRotate){
        if(longer) {
            if (startGoingLeft) {

                while (avgPosActual() < 450) {
                    setPower(0.2, 0.9, 0.9, 0.2);
                }
                simpleBrake();

                while (avgPosActual() < 250 && (color.getNormalizedColors().red < baseRed && color.getNormalizedColors().blue < baseBlue)) {
                    setPower(0.6, 0, 0, 0.6);
                }
                simpleBrake();


            } else {


                while (avgPosActual() < 450) {
                    setPower(0.9, 0.2, 0.2, 0.9);
                }
                simpleBrake();

                while (avgPosActual() < 250 && (color.getNormalizedColors().red < baseRed && color.getNormalizedColors().blue < baseBlue)) {
                    setPower(0, 0.6, 0.6, 0);
                }
                simpleBrake();





            }

            if(toRotate) {
                simpleMoveToPosition(-30, MovementType.ROTATE, 0.3);
            }else{
                simpleMoveToPosition(20, MovementType.ROTATE, 0.3);

            }
        }else{
            if (startGoingLeft) {

                while (avgPosActual() < 350) {
                    setPower(0.2, 0.9, 0.9, 0.2);
                }
                simpleBrake();

                while (avgPosActual() < 250 && (color.getNormalizedColors().red < baseRed && color.getNormalizedColors().blue < baseBlue)) {
                    setPower(0.6, 0, 0, 0.6);
                }
                simpleBrake();


            } else {


                while (avgPosActual() < 350) {
                    setPower(0.9, 0.2, 0.2, 0.9);
                }
                simpleBrake();

                while (avgPosActual() < 250 && (color.getNormalizedColors().red < baseRed && color.getNormalizedColors().blue < baseBlue)) {
                    setPower(0, 0.6, 0.6, 0);
                }
                simpleBrake();


            }

            if(toRotate) {
                //Changed for blue left PID (was -50)
                simpleMoveToPosition(-35, MovementType.ROTATE, 0.3);
            }else{
                //Change this later (4 red left)
                simpleMoveToPosition(-50, MovementType.ROTATE, 0.3);
            }
        }


    }

    public void autoDiagonals(boolean startGoingLeft, boolean longer, DiagonalPath auto){
        if(longer) {
            if (startGoingLeft) {

                while (avgPosActual() < 450) {
                    setPower(0.2, 0.9, 0.9, 0.2);
                }
                simpleBrake();

                while (avgPosActual() < 250 && (color.getNormalizedColors().red < baseRed && color.getNormalizedColors().blue < baseBlue)) {
                    setPower(0.6, 0, 0, 0.6);
                }
                simpleBrake();


            } else {


                while (avgPosActual() < 450) {
                    setPower(0.9, 0.2, 0.2, 0.9);
                }
                simpleBrake();

                while (avgPosActual() < 250 && (color.getNormalizedColors().red < baseRed && color.getNormalizedColors().blue < baseBlue)) {
                    setPower(0, 0.6, 0.6, 0);
                }
                simpleBrake();





            }

            if(auto == DiagonalPath.REDLEFT) {
                simpleMoveToPosition(-35, MovementType.ROTATE, 0.3);
            }else if(auto !=DiagonalPath.BLUERIGHT){
                simpleMoveToPosition(20, MovementType.ROTATE, 0.3);

            }
        }else{
            if (startGoingLeft) {

                while (avgPosActual() < 350) {
                    setPower(0.2, 0.9, 0.9, 0.2);
                }
                simpleBrake();

                while (avgPosActual() < 250 && (color.getNormalizedColors().red < baseRed && color.getNormalizedColors().blue < baseBlue)) {
                    setPower(0.6, 0, 0, 0.6);
                }
                simpleBrake();


            } else {


                while (avgPosActual() < 350) {
                    setPower(0.9, 0.2, 0.2, 0.9);
                }
                simpleBrake();

                while (avgPosActual() < 250 && (color.getNormalizedColors().red < baseRed && color.getNormalizedColors().blue < baseBlue)) {
                    setPower(0, 0.6, 0.6, 0);
                }
                simpleBrake();


            }

            if(auto == DiagonalPath.BLUELEFT) {
                //Changed for blue left PID (was -50)
                simpleMoveToPosition(-35, MovementType.ROTATE, 0.3);
            }else if(auto == DiagonalPath.REDLEFT){
                //Change this later (4 red left)
                simpleMoveToPosition(-60, MovementType.ROTATE, 0.3);
            }
        }


    }

    public double avgPosActual(){
        return (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition())
                + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) / 4;
    }

    public void addToLoggingString(String add){
        loggingString += (add + "\n");
    }

    public void writeLoggerToFile(){
        try{
            PrintStream toFile = new PrintStream(loggingFile);
            toFile.println(loggingString);
        }catch(FileNotFoundException e){
            e.printStackTrace();
        }
    }
}