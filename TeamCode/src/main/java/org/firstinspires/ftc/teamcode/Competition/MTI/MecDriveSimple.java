package org.firstinspires.ftc.teamcode.Competition.MTI;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Vector2D;

public class MecDriveSimple {
    DcMotorEx fl, fr, bl, br;
    Localizer odometry;
    Telemetry telemetry;
    BNO055IMU imu;

    private double wheelCircumference = 2 * 1.89 * Math.PI, pX = 50, pY = 0, pHeading = 0, dX = 0, dY = 0, dHeading = 0;

    public MecDriveSimple(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        odometry = new Localizer(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //TODO: Change the deviceName for each
        fl = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        br = hardwareMap.get(DcMotorEx.class, "BackRightDrive");




        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void update(){
        odometry.updatePosition();
    }

    public int leftTics(){
        return odometry.leftTics();
    }

    public int rightTics(){
        return odometry.rightTics();
    }

    public int auxTics(){
        return odometry.auxTics();
    }

    public double getX(){
        return odometry.getX();
    }

    public double getY(){
        return odometry.getY();
    }

    public double getAngle(){
        return odometry.getHeading();
    }

    public double getFirstAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    private double feetToCM(double feet){
        return feet * 30.48;
    }

    /**
     * Goes to targeted position using dead wheels
     * Can receive a delay for strafing and rotating (based on linear movement)
     * Has a limiter to the power and a kickout for the loop
     * @param xfeet
     * @param yfeet
     * @param heading
     * @param delayRotateTics
     * @param delayStrafeTics
     * @param kickout
     * @param limiter
     */
    public void odometryPID(double xfeet, double yfeet, double heading, int delayRotateTics, int delayStrafeTics, double kickout, int limiter){
        double firstX = odometry.getX();

        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        double actualStartTime = startTime;

        double xTarget = feetToCM(xfeet);
        double yTarget = feetToCM(yfeet);

        //TODO: make current values for x, y, and heading

        double xError = xTarget - odometry.getX();
        double yError = yTarget - odometry.getY();
        double headingError = heading - odometry.getHeading();

        double xPreviousError = 0;
        double yPreviousError = 0;
        double headingPreviousError = 0;

        double flVel = 0;
        double frVel = 0;
        double blVel = 0;
        double brVel = 0;

        while((Math.abs(xError) > 0.5 || Math.abs(yError) > 0.5 || Math.abs(headingError) > 0.5) && (time.seconds() - actualStartTime) < kickout){

            xError = xTarget - odometry.getX();
            yError = yTarget - odometry.getY();
            headingError = heading - odometry.getHeading();

            double currentTime = time.seconds();

            double xDerivative = (xError - xPreviousError)/(currentTime - startTime);
            double yDerviative = (yError - yPreviousError)/(currentTime - startTime);
            double headingDerivative = (headingError - headingPreviousError)/(currentTime - startTime);



            //Will have four different powers for each wheel
            double xPower = (pX * xError) + (dX * xDerivative);
            double yPower = (pY * yError) + (dY * yDerviative);
            double headingPower = (pHeading * headingError) + (dHeading * headingDerivative);

            //if there is a delay then make the yPower and headingPower take effect later
            //if(!(Current XPosition > Starting XPosition + delayStrafeTics)) -> set yPower to 0
            //if(!(Current XPosition > Starting XPosition + delayRotateTics)) -> set headingPower to 0

            if(!(odometry.getX() > (firstX + delayStrafeTics))){
                yPower = 0;
            }

            if(!(odometry.getX() > (firstX + delayRotateTics))){
                headingPower = 0;
            }

            //When going positive, need to figure out which direction wheels will go
            flVel += xPower - yPower + headingPower;
            frVel += xPower - yPower - headingPower;
            blVel += xPower + yPower + headingPower;
            brVel += xPower + yPower - headingPower;

            if(Math.abs(flVel) > limiter){
                if(flVel < 0){
                    flVel = -limiter;
                }else{
                    flVel = limiter;
                }
            }

            if(Math.abs(frVel) > limiter){
                if(frVel < 0){
                    frVel = -limiter;
                }else{
                    frVel = limiter;
                }
            }

            if(Math.abs(blVel) > limiter){
                if(blVel < 0){
                    blVel = -limiter;
                }else{
                    blVel = limiter;
                }
            }

            if(Math.abs(brVel) > limiter){
                if(brVel < 0){
                    brVel = -limiter;
                }else{
                    brVel = limiter;
                }
            }

            setVelocity(flVel, frVel, blVel, brVel);

            startTime = currentTime;
            xPreviousError = xError;
            yPreviousError = yError;
            headingPreviousError = headingError;

            odometry.updatePosition();



        }


        setVelocity(0,0,0,0);




    }

    public int getFLPosition(){
        return fl.getCurrentPosition();
    }

    public int getFRPosition(){
        return fr.getCurrentPosition();

    }

    public int getBLPosition(){
        return bl.getCurrentPosition();

    }

    public int getBRPosition(){
        return br.getCurrentPosition();

    }


    public void setPower(Vector2D velocity, double turnValue, boolean isSwapped) {
        turnValue = -turnValue;
        double direction = velocity.getDirection();


        double power = velocity.magnitude();

        double angle = direction + 3 * Math.PI / 4.0;
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        if (!isSwapped) {
            setPower((power * sin - turnValue), (power * cos + turnValue),
                    (power * cos - turnValue), (power * sin + turnValue));
        } else {
            setPower(-(power * sin - turnValue), -(power * cos + turnValue),
                    -(power * cos - turnValue), -(power * sin + turnValue));
        }
    }

    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(-flPow);
        fr.setPower(frPow);
        bl.setPower(-blPow);
        br.setPower(brPow);
    }

    public void setVelocity(double flPow, double frPow, double blPow, double brPow) {
        fl.setVelocity(-flPow);
        fr.setVelocity(frPow);
        bl.setVelocity(-blPow);
        br.setVelocity(brPow);
    }

    public void goToPosition(double flPow, double frPow, double blPow, double brPow, int tics) {
        //fl fr bl br
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // won't work for turns, only forward and backward
        int position = (int)(fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() +
                br.getCurrentPosition()) / 4;

        telemetry.addData("motorPosition", position);
        telemetry.update();

        while (Math.abs(position - tics) > 0) {
            setPower(flPow, frPow, blPow, brPow);
        }

        setPower(0, 0, 0, 0);

    }

    public void resetEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //Method used for testing purposes
    public int getPosition(){
        int position = (int)(fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() +
                br.getCurrentPosition()) / 4;
        return position;
    }

}