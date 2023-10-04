package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

public class ScoringSystem {
    public DcMotorEx lLift, rLift;
    public Servo grabber;
    public Servo lIntakeLift, rIntakeLift;
    public CRServo lIntake, rIntake;
    public DistanceSensor distance;
    ServoImplEx rLinkage, lLinkage;
    public ScoringMode height;
    private boolean grabbing, linkageUp, extended;
    private int coneStack, rightPreviousError, leftPreviousError, liftTarget;
    private double startTime, currentTime;
    Telemetry telemetry;
    ElapsedTime time;


    PIDFCoefficients pidf = new PIDFCoefficients(0, 0, 0, 0); //Old P = 0.000475

    File file = AppUtil.getInstance().getSettingsFile("motion.txt");
    String composite = "";


    public enum ScoringMode{
        HIGH,
        MEDIUM,
        LOW,
        ULTRA
    }

    public ScoringSystem(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime time) {
        coneStack = 1;
        height = ScoringMode.LOW;
        extended = false;

        this.time = time;
        this.telemetry = telemetry;

        // Lift
        rLift = hardwareMap.get(DcMotorEx.class, "RightLift");
        lLift = hardwareMap.get(DcMotorEx.class, "LeftLift");

        rLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        rLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Linkage
        lLinkage = hardwareMap.get(ServoImplEx.class, "LeftLinkage");
        rLinkage = hardwareMap.get(ServoImplEx.class, "RightLinkage");

        lLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));

        grabber =  hardwareMap.get(Servo.class, "Grabber");

        // Intake
        lIntakeLift = hardwareMap.get(Servo.class, "LeftIntakeLift");
        rIntakeLift = hardwareMap.get(Servo.class, "RightIntakeLift");

        lIntake = hardwareMap.get(CRServo.class, "leftIntake");
        lIntake = hardwareMap.get(CRServo.class, "rightIntake");

        setLinkagePosition(Constants.linkageDown);
        setIntakePower(0);
        setIntakeLiftPos(0);

    }



    public int getConeStack(){
        return coneStack;
    }

    public void lowerConeStack(){
        if(coneStack - 1 > 0) {
            coneStack -= 1;
        }
    }

    public void raiseConeStack(){
        if(coneStack + 1 < 6) {
            coneStack += 1;
        }
    }

    public void setConeStack(int height){
        coneStack = height;

    }

    //TODO: tune this
    public void setLinkageConeStack(boolean logistic){
        if(logistic){
            if(coneStack == 5){
                setLinkagePositionLogistic(0.31, 500);
            }else if(coneStack == 4){
                setLinkagePositionLogistic(0.26, 500);
            }else if(coneStack == 3){
                setLinkagePositionLogistic(0.23, 500);
            }else if(coneStack == 2){
                //setLinkagePositionLogistic(Constants.linkageDownV2 + 0.1, 300);
                setLinkagePositionLogistic(0.21, 500);
            }else if(coneStack == 1){
                //setLinkagePositionLogistic(Constants.linkageDownV2 + 0.1, 300);
                setLinkagePositionLogistic(0.15, 500);
            }
        }else {
            if (coneStack == 5) {
                setLinkagePosition(0.31);
            } else if (coneStack == 4) {
                setLinkagePosition(0.26);
            } else if (coneStack == 3) {
                setLinkagePosition(0.23);
            } else if (coneStack == 2) {
                //setLinkagePosition(Constants.linkageDownV2 + 0.1);
                setLinkagePosition(0.21);
            } else if (coneStack == 1) {
                //setLinkagePosition(Constants.linkageDownV2 + 0.1);
                setLinkagePosition(0.15);
            }
        }
    }

    public boolean isExtended() {
        return extended;
    }

    public void setExtended(boolean extended) {
        this.extended = extended;
    }

    public void setScoringMode(ScoringMode height){
        this.height = height;
    }

    public ScoringMode getScoringMode(){
        return height;
    }



    //TODO: Fix this
    public int getHeight(){
       if(height == ScoringMode.HIGH){
           return Constants.LIFT_HIGH;
       }else if(height == ScoringMode.MEDIUM){
           return Constants.LIFT_MEDIUM;
       }else if(height == ScoringMode.LOW){
           return Constants.LIFT_LOW;
       }

       return 0;
    }

    public void setPower(double power) {
        rLift.setPower(power);
        lLift.setPower(-power);
    }



    public void setPower(double rightPower, double leftPower){
        rLift.setPower(rightPower);
        lLift.setPower(-leftPower);
    }

    public void setVelocity(double rightPower, double leftPower){
        rLift.setVelocity(rightPower);
        lLift.setVelocity(-leftPower);
    }

    public int getLeftEncoderPos() {
        return lLift.getCurrentPosition();
    }

    public int getRightEncoderPos() {
        return rLift.getCurrentPosition();
    }

    public void changeMode(ScoringMode score){
        height = score;
    }

    public void commandAutoGoToPosition(){
        if(height == ScoringMode.HIGH /*|| height == ScoringMode.ULTRA*/){
            setLiftTarget(Constants.LIFT_HIGH);

        }else if(height == ScoringMode.MEDIUM){
            setLiftTarget(Constants.LIFT_MEDIUM);

        }else if(height == ScoringMode.LOW){
            setLiftTarget(Constants.LIFT_LOW);

        }

        extended = true;
    }



    public void moveToPosition(int tics, double power, double kickout){

        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();

        int rLiftPos = -1 * rLift.getCurrentPosition();
        int lLiftPos = lLift.getCurrentPosition();


        if(tics < ((rLiftPos + lLiftPos) / 2)){
            power *= -1;
        }

        double rightPower = power;
        double leftPower = power;





        //Dont know if need the != condition
        //if ((tics == 0 && rLiftPos != 0 && lLiftPos != 0)) {

        //TODO: Check if logic for encoder positions works

        if(power > 0) {
            while ((time.seconds() - startTime) < kickout && (rLiftPos < tics || lLiftPos < tics)) {

                //TODO: figure out if we need to negate either of them

                if (rLiftPos >= tics) {
                    rightPower = 0;
                }
                if (lLiftPos >= tics) {
                    leftPower = 0;
                }


                rLiftPos = -1 * getRightEncoderPos();
                lLiftPos = getLeftEncoderPos();

                setPower(rightPower, leftPower);


            }
        }else{
            while ((time.seconds() - startTime) < kickout &&   (rLiftPos > tics || lLiftPos > tics)) {

                //TODO: figure out if we need to negate either of them

                if (rLiftPos <= tics) {
                    rightPower = 0;
                }

                if (lLiftPos <= tics) {
                    leftPower = 0;
                }


                rLiftPos = -1 * getRightEncoderPos();
                lLiftPos = getLeftEncoderPos();

                setPower(rightPower, leftPower);


            }
        }

        setPower(0);

    }


    public int moveToPosition(int tics, double power, double kickout, boolean distanceSensor){

        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();

        int rLiftPos = -1 * rLift.getCurrentPosition();
        int lLiftPos = -1 * lLift.getCurrentPosition();


        if(tics < ((rLiftPos + lLiftPos) / 2)){
            power *= -1;
        }

        double rightPower = power;
        double leftPower = power;

        PrintStream ps = null;
        try {
            ps = new PrintStream(file);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }


        //Dont know if need the != condition
        //if ((tics == 0 && rLiftPos != 0 && lLiftPos != 0)) {

        //TODO: Check if logic for encoder positions works
        double counter = 0;
        double prevmillis = 0;


        if(power > 0) {
            while ((time.seconds() - startTime) < 2.25 && (rLiftPos < tics || lLiftPos < tics) && distance.getDistance(DistanceUnit.CM) > 15) {

                //TODO: figure out if we need to negate either of them

                /*
                if (rLiftPos >= tics) {
                    rightPower = 0;
                }

                if (lLiftPos >= tics) {
                    leftPower = 0;
                }

                 */
                double millis = time.milliseconds();
                composite += "(" + counter + " , " + millis + ")\n";
                counter++;
                prevmillis = millis;

                rLiftPos = -1 * rLift.getCurrentPosition();
                lLiftPos = -1 * lLift.getCurrentPosition();

                setPower(rightPower, leftPower);

            }
        }else{
            while ((time.seconds() - startTime) < 2.5 &&   (rLiftPos > tics || lLiftPos > tics) && distance.getDistance(DistanceUnit.CM) > 15) {

                //TODO: figure out if we need to negate either of them

                /*
                if (rLiftPos <= tics) {
                    rightPower = 0;
                }

                if (lLiftPos <= tics) {
                    leftPower = 0;
                }

                 */


                rLiftPos = -1 * rLift.getCurrentPosition();
                lLiftPos = -1 * lLift.getCurrentPosition();

                setPower(rightPower, leftPower);

            }
        }
        composite += "Delta Avg: " + (prevmillis) / counter;
        ps.println(composite);

        setPower(0);

        return Math.abs(rLift.getCurrentPosition());

    }

    public void newLiftPID(int tics, double power){


        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        double intStartTime = time.milliseconds();


        int rightIntegralSum = 0;
        int leftIntegralSum = 0;


        int rLiftPos = -1 * rLift.getCurrentPosition();
        int lLiftPos = lLift.getCurrentPosition();

        int leftError = tics - lLiftPos;
        int rightError = tics - rLiftPos;

        int leftPreviousError = leftError;
        int rightPreviousError = rightError;



        //Dont know if need the != condition
        //if ((tics == 0 && rLiftPos != 0 && lLiftPos != 0)) {

        //TODO: Check if logic for encoder positions works

        while ((time.seconds() - startTime) < 1.25 && Math.abs(leftError) > 2 && Math.abs(leftError) > 2) {

            //TODO: figure out if we need to negate either of them


            rLiftPos = -1 * rLift.getCurrentPosition();
            lLiftPos = lLift.getCurrentPosition();

            telemetry.addData("rLift", rLiftPos);
            telemetry.addData("lLift", lLiftPos);


            double currentTime = time.milliseconds();


            leftError = tics - lLiftPos;
            rightError = tics - rLiftPos;

            telemetry.addData("leftError", leftError);
            telemetry.addData("rightError", rightError);


            leftIntegralSum += (0.5 * (leftError + leftPreviousError) * (currentTime - intStartTime));
            rightIntegralSum += (0.5 * (rightError + rightPreviousError) * (currentTime - intStartTime));

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

            telemetry.addData("rightIntegral", rightIntegralSum);
            telemetry.addData("leftIntegral", leftIntegralSum);

            double leftDerivative = (leftError - leftPreviousError) / (currentTime - intStartTime);
            double rightDerivative = (rightError - rightPreviousError) / (currentTime - intStartTime);

            telemetry.addData("rightDerivative", rightDerivative);
            telemetry.addData("leftDerivative", leftDerivative);

            double leftPower = ((leftError * pidf.p) + (leftIntegralSum * pidf.i) + (leftDerivative * pidf.d));
            double rightPower = ((rightError * pidf.p) + (rightIntegralSum * pidf.i) + (rightDerivative * pidf.d));

            telemetry.addData("rightPower", rightPower);
            telemetry.addData("leftPower", leftPower);





            setPower(rightPower, leftPower);

            telemetry.update();


            intStartTime = currentTime;
            leftPreviousError = leftError;
            rightPreviousError = rightError;



        }

        setPower(0);


    }




    public void setTimeServoPosLogistic(double target, int time) {
        ElapsedTime timer = new ElapsedTime();
        double startTime = timer.milliseconds();
        double endTime = startTime + time;
        int timeResolution = 20;
        double start = getLeftLinkage();
        double b = (1 - start)/ start;
        double k = (Math.log(((start/target) * (1 - target))/(1 - start)) / time);


        while(startTime < endTime) {
            setLinkagePosition(logisticFormula(start, target, time, b, k));
            startTime = timer.milliseconds();
            sleep(timeResolution);
        }
        setLinkagePosition(target);
    }

    private double logisticFormula(double start, double end, int time, double b, double k){




        return 1 / (1 + (b * Math.pow(Math.E, (k * (time / 1000)))));
    }


    public void setLinkagePosition(double position){
        //TODO: tune position values
        rLinkage.setPosition(position);
        lLinkage.setPosition(position);

        telemetry.addData("lift thing", "potato");
        telemetry.update();
    }

    public void setLinkagePositionSpecial(double position, int liftTics){
        //TODO: tune position values
        while(Math.abs(getRightEncoderPos()) < liftTics){

        }

        rLinkage.setPosition(position);
        lLinkage.setPosition(position);

        telemetry.addData("lift thing", "potato");
        telemetry.update();
    }

    public void setLinkagePositionLogistic(double target, int sleepTime) {

        ElapsedTime time = new ElapsedTime();
        int resolution = 100;
        double step = 4.0 / resolution;
        double start = getLeftLinkage();
        double startX = -2.0;

        double startTime = time.milliseconds();


        for(int i = 0; i < resolution; i++) {
            setLinkagePosition(logistic(startX, start, target));
            startX += step;
            sleep(sleepTime / resolution);

            if(time.milliseconds() - startTime > (3 * sleepTime)){
                break;
            }

            telemetry.addData("lift thing", "potato");
            telemetry.update();
        }
        setLinkagePosition(target);
    }

    public void setLinkagePositionLogistic(double target, int sleepTime, int resolution) {
        ElapsedTime timer = new ElapsedTime();
        double step = 4.0 / resolution;
        double start = getLeftLinkage();
        double startX = -2.0;
        double startLoopTime;
        double loopTime;
        for(int i = 0; i < resolution; i++) {
            startLoopTime = timer.milliseconds();
            setLinkagePosition(logistic(startX, start, target));
            startX += step;
            loopTime = timer.milliseconds() - startLoopTime;
            if ((double)sleepTime / resolution > loopTime) {
                sleep((long)(sleepTime / resolution - loopTime));
            } else {
                if (timer.milliseconds() > sleepTime) {
                    break;
                }
            }
        }
        setLinkagePosition(target);
    }

    public double getRightLinkage(){
        return rLinkage.getPosition();
    }

    public double getLeftLinkage(){
        return lLinkage.getPosition();
    }


    public void linkageUpAndDown(boolean up){
        if(up) {
            setLinkagePosition(Constants.linkageUp);
        }else{
            setLinkagePosition(Constants.linkageDown);
        }
    }

    public void reset(){
        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }




    public void newLiftPD(int tics, double limiter, double kickout){

        boolean initialErrorIsNegative = false;


        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        double actualStartTime = startTime;



        //TODO: check if we need to negate any

        int rightPos = -1 * getRightEncoderPos();
        int leftPos = getLeftEncoderPos();

        int rightError = tics - rightPos;
        int leftError = tics - leftPos;


        if(leftError < 0){
            initialErrorIsNegative = true;
        }

        int rightPreviousError = 0;
        int leftPreviousError = 0;


        while(Math.abs(rightError) > 2 && Math.abs(leftError) > 2 && (time.seconds() - actualStartTime) < kickout){
            //telemetry.addData("target", tics);


            //TODO: check if we need to negate any

            rightPos = -1 * getRightEncoderPos();
            leftPos = getLeftEncoderPos();




            rightError = tics - rightPos;
            leftError = tics - leftPos;

            if((initialErrorIsNegative && (rightError > 0 || leftError > 0)) || (!initialErrorIsNegative && (rightError < 0 || leftError < 0))){
                break;
            }

            double currentTime = time.seconds();



            double rightDerivative = (rightError - rightPreviousError)/(currentTime - startTime);
            double leftDerivative = (leftError - leftPreviousError)/(currentTime - startTime);


            double rightPower = ((pidf.p * rightError) + (pidf.d * rightDerivative) + (pidf.f * Math.abs(getRightEncoderPos())));
            double leftPower = ((pidf.p * leftError) + (pidf.d * leftDerivative) + (pidf.f * Math.abs(getRightEncoderPos())));

            if(Math.abs(rightPower) > limiter){
                if(rightPower < 0){
                    rightPower = -1 * limiter;
                }else{
                    rightPower = limiter;
                }
            }

            if(Math.abs(leftPower) > limiter){
                if(leftPower < 0){
                    leftPower = -1 * limiter;
                }else{
                    leftPower = limiter;
                }
            }


            //telemetry.addData("rightError", rightError);
            //telemetry.addData("leftError", leftError);

            setPower(rightPower, leftPower);


            startTime = currentTime;
            rightPreviousError = rightError;
            leftPreviousError = leftError;






        }

        //setPower(0,0,0,0);
        setPower(0);


    }


    public void newLiftPIDUpdate(double limiter, boolean high){
        currentTime = time.seconds();

        int rightPos = -1 * getRightEncoderPos();
        int leftPos = getLeftEncoderPos();


        //telemetry.addData("rightPos", rightPos);
        //telemetry.addData("leftPos", leftPos);

        int rightError = liftTarget - rightPos;
        int leftError = liftTarget - leftPos;


        double rightDerivative = (rightError - rightPreviousError)/(currentTime - startTime);
        double leftDerivative = (leftError - leftPreviousError)/(currentTime - startTime);

        //telemetry.addData("rightDerivative", rightDerivative);
        //telemetry.addData("leftDerivative", leftDerivative);

        double rightPower = ((pidf.p * rightError) + (pidf.d * rightDerivative) + (pidf.f * Math.abs(getRightEncoderPos())));
        double leftPower = ((pidf.p * leftError) + (pidf.d * leftDerivative) + (pidf.f * Math.abs(getRightEncoderPos())));

        if(Math.abs(rightPower) > limiter){
            if(rightPower < 0){
                rightPower = -1 * limiter;
            }else{
                rightPower = limiter;
            }
        }

        if(Math.abs(leftPower) > limiter){
            if(leftPower < 0){
                leftPower = -1 * limiter;
            }else{
                leftPower = limiter;
            }
        }

        if(high){
            rightPower = 1;
            leftPower = 1;
        }



        setPower(rightPower, leftPower);


        startTime = currentTime;
        rightPreviousError = rightError;
        leftPreviousError = leftError;






    }



    public void setLiftTarget(int target){
        liftTarget = target;
    }

    public int getLiftTarget(){
        return liftTarget;
    }


    public void setGrabberPosition(double position){
        grabber.setPosition(position);
    }

    public double getGrabberPosition(){
        return grabber.getPosition();
    }


    public void grabberOpenAndClose(boolean close){
        if(close) {
            setGrabberPosition(Constants.grabbing);
        }else{
            setGrabberPosition(Constants.open);
        }
    }

    public static double logistic(double x, double lower, double upper) {
        double k = 2;
        double x0 = 0;
        upper -= lower;
        return (upper / (1 + Math.pow(Math.E, -k * ( x - x0 )))) + lower;
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void setGrabbing(boolean grabbing) {
        this.grabbing = grabbing;
    }

    public void setLinkageUp(boolean linkageUp) {
        this.linkageUp = linkageUp;
    }

    public boolean isGrabbing() {
        return grabbing;
    }

    public boolean isLinkageUp() {
        return linkageUp;
    }

    public void addToLoggingString(String add){
        composite += (add + "\n");
    }

    public void setIntakePower(double power) {
        lIntake.setPower(-power);
        rIntake.setPower(power * 0.8);
    }

    public void setIntakeLiftPos(double pos) {
        lIntakeLift.setPosition(pos);
        rIntakeLift.setPosition(pos);
    }

    public void writeLoggerToFile(){
        try{
            PrintStream toFile = new PrintStream(file);
            toFile.print(composite);
        }catch(FileNotFoundException e){
            e.printStackTrace();
        }
    }

}
