package org.firstinspires.ftc.teamcode.V2.NewSubsystem;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;

public class ScoringSystemV2 {
    DcMotorEx lLift, rLift;
    public Servo grabber;
    ServoImplEx rLinkage, lLinkage;
    public ScoringMode height;
    private boolean grabbing, linkageUp, extended;
    //Constants constants;
    private int coneStack;
    Telemetry telemetry;
    PIDCoefficients lift = new PIDCoefficients(0, 0, 0);




    public enum ScoringMode{
        HIGH,
        MEDIUM,
        LOW,
        ULTRA
    }

    public ScoringSystemV2(HardwareMap hardwareMap/*Constants constants*/) {

        //coneStack = 2;
        height = ScoringMode.HIGH;
        extended = false;

        rLift = hardwareMap.get(DcMotorEx.class, "RightLift");
        lLift = hardwareMap.get(DcMotorEx.class, "LeftLift");

        rLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        rLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lLinkage = hardwareMap.get(ServoImplEx.class, "LeftLinkage");
        rLinkage = hardwareMap.get(ServoImplEx.class, "RightLinkage");

        lLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));

        grabber =  hardwareMap.get(Servo.class, "Grabber");


        setLinkagePosition(Constants.linkageDownV2);
        grabber.setPosition(Constants.open);

    }

    public ScoringSystemV2(HardwareMap hardwareMap, /*//Constants constants,*/ Telemetry telemetry) {
        this.telemetry = telemetry;

        coneStack = 2;
        height = ScoringMode.HIGH;
        extended = false;

        rLift = hardwareMap.get(DcMotorEx.class, "RightLift");
        lLift = hardwareMap.get(DcMotorEx.class, "LeftLift");

        rLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        rLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lLinkage = hardwareMap.get(ServoImplEx.class, "LeftLinkage");
        rLinkage = hardwareMap.get(ServoImplEx.class, "RightLinkage");

        lLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));

        grabber = hardwareMap.get(Servo.class, "Grabber");


        setLinkagePosition(Constants.linkageDownV2);
        //setLinkagePosition(0.8);
        grabber.setPosition(Constants.open);

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

    //TODO: tune this
    public void setLinkageConeStack(boolean logistic){
        if(logistic){
            if(coneStack == 5){
                setLinkagePositionLogistic(0.288, 500);
            }else if(coneStack == 4){
                setLinkagePositionLogistic(0.2625, 500);
            }else if(coneStack == 3){
                setLinkagePositionLogistic(0.2215, 500);
            }else if(coneStack == 2){
                setLinkagePositionLogistic(0.1885, 500);
            }else if(coneStack == 1){
                setLinkagePositionLogistic(Constants.linkageDownV2, 500);
            }
        }else {
            if (coneStack == 5) {
                setLinkagePosition(0.288);
            } else if (coneStack == 4) {
                setLinkagePosition(0.2625);
            } else if (coneStack == 3) {
                setLinkagePosition(0.2215);
            } else if (coneStack == 2) {
                setLinkagePosition(0.1885);
            } else if (coneStack == 1) {
                setLinkagePosition(Constants.linkageDownV2);
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
           return 1400;
       }else if(height == ScoringMode.MEDIUM){
           return 1000;
       }else if(height == ScoringMode.LOW){
           return 250;
       }

       return 0;
    }

    public void setPower(double power) {
        rLift.setPower(-power);
        lLift.setPower(power);
    }

    public void setPower(double rightPower, double leftPower){
        rLift.setPower(-rightPower);
        lLift.setPower(leftPower);
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

    //TODO: fix this
    public void autoGoToPosition(){
        if(height == ScoringMode.HIGH /*|| height == ScoringMode.ULTRA*/){
            moveToPosition(1400, 1, 2.25);

        }else if(height == ScoringMode.MEDIUM){
            moveToPosition(650, 1);


        }else if(height == ScoringMode.LOW){
            moveToPosition(150, 1);

        }

        extended = true;
    }

    //TODO: fix this
    public void autoGoToPosition(ScoringMode height) {
        this.height = height;
        if (height == ScoringMode.HIGH || height == ScoringMode.ULTRA) {
            moveToPosition(1400, 1);

        } else if (height == ScoringMode.MEDIUM) {

            //TODO: Find tic value
            moveToPosition(650, 1);


        } else if (height == ScoringMode.LOW) {
            //TODO: Find tic value
            moveToPosition(150, 1);

        }


    }






    public void moveToPosition(int tics, double power){

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
            while ((time.seconds() - startTime) < 2.25 && (rLiftPos < tics || lLiftPos < tics)) {

                //TODO: figure out if we need to negate either of them

                if (rLiftPos >= tics) {
                    rightPower = 0;
                } else if (lLiftPos >= tics) {
                    leftPower = 0;
                }


                rLiftPos = -1 * rLift.getCurrentPosition();
                lLiftPos = lLift.getCurrentPosition();

                setPower(rightPower, leftPower);


            }
        }else{
            while ((time.seconds() - startTime) < 2.5 &&   (rLiftPos > tics || lLiftPos > tics)) {

                //TODO: figure out if we need to negate either of them

                if (rLiftPos <= tics) {
                    rightPower = 0;
                } else if (lLiftPos <= tics) {
                    leftPower = 0;
                }


                rLiftPos = -1 * rLift.getCurrentPosition();
                lLiftPos = lLift.getCurrentPosition();

                setPower(rightPower, leftPower);


            }
        }

        setPower(0);

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
                } else if (lLiftPos >= tics) {
                    leftPower = 0;
                }


                rLiftPos = -1 * rLift.getCurrentPosition();
                lLiftPos = lLift.getCurrentPosition();

                setPower(rightPower, leftPower);


            }
        }else{
            while ((time.seconds() - startTime) < 4 &&   (rLiftPos > tics || lLiftPos > tics)) {

                //TODO: figure out if we need to negate either of them

                if (rLiftPos <= tics) {
                    rightPower = 0;
                } else if (lLiftPos <= tics) {
                    leftPower = 0;
                }


                rLiftPos = -1 * rLift.getCurrentPosition();
                lLiftPos = lLift.getCurrentPosition();

                setPower(rightPower, leftPower);


            }
        }

        setPower(0);

    }



    public int[] moveToPosition(int tics, double power, boolean isPID){

        int[] intSums = {0, 0};


        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        double intStartTime = time.seconds();

        int rLiftPos = -1 * rLift.getCurrentPosition();
        int lLiftPos = lLift.getCurrentPosition();

        int leftPreviousError = Math.abs(tics - lLiftPos);
        int rightPreviousError = Math.abs(tics - rLiftPos);



        if(tics < ((rLiftPos + lLiftPos) / 2)){
            power *= -1;
        }

        double rightPower = power;
        double leftPower = power;





        //Dont know if need the != condition
        //if ((tics == 0 && rLiftPos != 0 && lLiftPos != 0)) {

        //TODO: Check if logic for encoder positions works

        if(power > 0) {
            while ((time.seconds() - startTime) < 3 && rLiftPos < tics || lLiftPos < tics) {

                //TODO: figure out if we need to negate either of them

                //left
                intSums[0] += tics - lLiftPos;

                //right
                intSums[1] += tics - rLiftPos;



                if (rLiftPos >= tics) {
                    rightPower = 0;
                } else if (lLiftPos >= tics) {
                    leftPower = 0;
                }


                rLiftPos = -1 * rLift.getCurrentPosition();
                lLiftPos = lLift.getCurrentPosition();

                telemetry.addData("rLift", rLiftPos);
                telemetry.addData("lLift", lLiftPos);

                double currentTime = time.seconds();

                int leftError = tics - lLiftPos;
                int rightError = tics - rLiftPos;

                telemetry.addData("leftError", leftError);
                telemetry.addData("rightError", rightError);

                //left
                intSums[0] += (0.5 * (leftError + leftPreviousError) * (currentTime - intStartTime));

                //right
                intSums[1] += (0.5 * (rightError + rightPreviousError) * (currentTime - intStartTime));


                telemetry.addData("rightIntegral", intSums[0]);
                telemetry.addData("leftIntegral", intSums[1]);





                if(intSums[0] > 20000){
                    intSums[0] = 20000;
                }else if(intSums[0] < -20000){
                    intSums[0] = -20000;
                }

                if(intSums[1] > 20000){
                    intSums[1] = 20000;
                }else if(intSums[1] < -20000){
                    intSums[1] = -20000;
                }


                setPower(rightPower, leftPower);

                telemetry.update();


                intStartTime = currentTime;
                leftPreviousError = leftError;
                rightPreviousError = rightError;


            }
        }else{
            while ((time.seconds() - startTime) < 3 && rLiftPos > tics || lLiftPos > tics) {

                //TODO: figure out if we need to negate either of them

                //left
                intSums[0] += tics - lLiftPos;

                //right
                intSums[1] += tics - rLiftPos;

                if (rLiftPos <= tics) {
                    rightPower = 0;
                } else if (lLiftPos <= tics) {
                    leftPower = 0;
                }


                rLiftPos = -1 * rLift.getCurrentPosition();
                lLiftPos = lLift.getCurrentPosition();

                setPower(rightPower, leftPower);


            }
        }

        setPower(0);

        return intSums;

    }

    public void moveToPIDPosition(int tics, double power){


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

        while ((time.seconds() - startTime) < 1.25 && Math.abs(leftError) > 2 || Math.abs(leftError) > 2) {

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

            double leftPower = ((leftError * lift.p) + (leftIntegralSum * lift.i) + (leftDerivative * lift.d));
            double rightPower = ((rightError * lift.p) + (rightIntegralSum * lift.i) + (rightDerivative * lift.d));

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
    }

    public void setLinkagePositionLogistic(double target, int sleepTime) {
        int resolution = 100;
        double step = 4.0 / resolution;
        double start = getLeftLinkage();
        double startX = -2.0;
        for(int i = 0; i < resolution; i++) {
            setLinkagePosition(logistic(startX, start, target));
            startX += step;
            sleep(sleepTime / resolution);
        }
        setLinkagePosition(target);
    }

    public void setLinkagePositionLogistic(double target, int sleepTime, int resolution) {
        double step = 4.0 / resolution;
        double start = getLeftLinkage();
        double startX = -2.0;
        for(int i = 0; i < resolution; i++) {
            setLinkagePosition(logistic(startX, start, target));
            startX += step;
            sleep(sleepTime / resolution);
        }
        setLinkagePosition(target);
    }

    public double getRightLinkage(){
        return rLinkage.getPosition();
    }

    public double getLeftLinkage(){
        return lLinkage.getPosition();
    }

    public void linkageAutomated(boolean up){
        if(up){
            setLinkagePosition(Constants.linkageUp);
        }else{
            setLinkagePosition(Constants.linkageDown);

        }
    }

    //TODO: Tune this
    public void shiftLinkagePosition(){
        double setHeight = getGrabberPosition() - 0.04;

        if(setHeight < 0.09){
            setHeight = 0.25;
        }else if(setHeight > 0.25){
            setHeight = 0.25;
        }

        setLinkagePosition(setHeight);
    }

    public void linkageUpAndDown(boolean up){
        if(up) {
            setLinkagePosition(Constants.linkageUp);
        }else{
            setLinkagePosition(Constants.linkageDown);
        }
    }

    public int getEncoderPosition(boolean right){
        if(right){
            return rLift.getCurrentPosition();
        }else{
            return lLift.getCurrentPosition();

        }
    }

    public void reset(){
        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public boolean isBusy(){
        return rLift.isBusy() && lLift.isBusy();
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

}
