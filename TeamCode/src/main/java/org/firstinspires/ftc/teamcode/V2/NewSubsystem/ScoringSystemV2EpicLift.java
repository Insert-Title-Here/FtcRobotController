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

public class ScoringSystemV2EpicLift {
    public DcMotorEx lLift1, rLift1, lLift2, rLift2;
    public Servo grabber;
    ServoImplEx rLinkage, lLinkage;
    public ScoringMode height;
    private boolean grabbing, linkageUp, extended;
    Constants constants;
    private int coneStack;
    Telemetry telemetry;
    PIDCoefficients pidf = new PIDCoefficients(0.0076, 0.0000275, 0.00025);




    public enum ScoringMode{
        HIGH,
        MEDIUM,
        LOW,
        ULTRA
    }

    public ScoringSystemV2EpicLift(HardwareMap hardwareMap, Constants constants) {


        coneStack = 1;
        height = ScoringMode.HIGH;
        extended = false;
        this.constants = constants;

        rLift1 = hardwareMap.get(DcMotorEx.class, "RightLift");
        lLift1 = hardwareMap.get(DcMotorEx.class, "LeftLift");

        rLift2 = hardwareMap.get(DcMotorEx.class, "RightLift2");
        lLift2 = hardwareMap.get(DcMotorEx.class, "LeftLift2");

        rLift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lLift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        rLift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lLift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        rLift1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lLift1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rLift2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lLift2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rLift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lLift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        rLift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lLift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



        lLinkage = hardwareMap.get(ServoImplEx.class, "LeftLinkage");
        rLinkage = hardwareMap.get(ServoImplEx.class, "RightLinkage");

        lLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));

        grabber =  hardwareMap.get(Servo.class, "Grabber");


        setLinkagePosition(Constants.linkageDownV2);
        grabber.setPosition(constants.open);

    }

    public ScoringSystemV2EpicLift(HardwareMap hardwareMap, Constants constants, Telemetry telemetry) {
        this.telemetry = telemetry;

        coneStack = 1;
        height = ScoringMode.HIGH;
        extended = false;
        this.constants = constants;

        rLift1 = hardwareMap.get(DcMotorEx.class, "RightLift");
        lLift1 = hardwareMap.get(DcMotorEx.class, "LeftLift");

        rLift2 = hardwareMap.get(DcMotorEx.class, "RightLift2");
        lLift2 = hardwareMap.get(DcMotorEx.class, "LeftLift2");

        rLift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lLift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        rLift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lLift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        rLift1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lLift1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rLift2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lLift2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rLift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lLift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        rLift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lLift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lLinkage = hardwareMap.get(ServoImplEx.class, "LeftLinkage");
        rLinkage = hardwareMap.get(ServoImplEx.class, "RightLinkage");

        lLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rLinkage.setPwmRange(new PwmControl.PwmRange(500, 2500));

        grabber = hardwareMap.get(Servo.class, "Grabber");


        setLinkagePosition(Constants.linkageDownV2);
        //setLinkagePosition(0.8);
        grabber.setPosition(constants.open);

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
                setLinkagePositionLogistic(0.2625, 300);
            }else if(coneStack == 4){
                setLinkagePositionLogistic(0.2215, 300);
            }else if(coneStack == 3){
                setLinkagePositionLogistic(0.1885, 300);
            }else if(coneStack == 2){
                setLinkagePositionLogistic(Constants.linkageDownV2, 300);
            }else if(coneStack == 1){
                setLinkagePositionLogistic(Constants.linkageDownV2, 300);
            }
        }else {
            if (coneStack == 5) {
                setLinkagePosition(0.2625);
            } else if (coneStack == 4) {
                setLinkagePosition(0.2215);
            } else if (coneStack == 3) {
                setLinkagePosition(0.1885);
            } else if (coneStack == 2) {
                setLinkagePosition(Constants.linkageDownV2);
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

    public double getI(){
        return pidf.i;
    }


    //TODO: Fix this
    public int getHeight(){
       if(height == ScoringMode.HIGH){
           return 1350;
       }else if(height == ScoringMode.MEDIUM){
           return 750;
       }else if(height == ScoringMode.LOW){
           return 250;
       }

       return 0;
    }

    public void setPower(double power) {
        rLift1.setPower(-power);
        //lLift1.setPower(power);

        rLift2.setPower(-power);
        //lLift2.setPower(power);
    }

    public void setPowerSingular(double power) {
        rLift1.setPower(-power);
        //lLift1.setPower(power);
    }

    public void setPower(double rightPower, double leftPower){
        rLift1.setPower(-rightPower);
        //lLift1.setPower(leftPower);

        rLift2.setPower(-rightPower);
        //lLift2.setPower(leftPower);
    }
    public int getLeftEncoderPos() {
        return lLift1.getCurrentPosition();
    }

    public int getRightEncoderPos() {
        return rLift1.getCurrentPosition();
    }

    public void changeMode(ScoringMode score){
        height = score;
    }

    //TODO: fix this
    public void autoGoToPosition(){
        if(height == ScoringMode.HIGH /*|| height == ScoringMode.ULTRA*/){
            moveToPosition(600, 0.85, 2.25);

        }else if(height == ScoringMode.MEDIUM){
            moveToPosition(400, 0.85);


        }else if(height == ScoringMode.LOW){
            moveToPosition(200, 0.85);

        }

        extended = true;
    }

    public void epicAutoGoToPosition(){
        if(height == ScoringMode.HIGH /*|| height == ScoringMode.ULTRA*/){
            newLiftPID(1350);

        }else if(height == ScoringMode.MEDIUM){
            newLiftPID(750);


        }else if(height == ScoringMode.LOW){
            newLiftPID(250);

        }

        extended = true;
    }

    //TODO: fix this
    public void autoGoToPosition(ScoringMode height) {
        this.height = height;
        if (height == ScoringMode.HIGH || height == ScoringMode.ULTRA) {
            moveToPosition(600, 1);

        } else if (height == ScoringMode.MEDIUM) {

            //TODO: Find tic value
            moveToPosition(400, 1);


        } else if (height == ScoringMode.LOW) {
            //TODO: Find tic value
            moveToPosition(200, 1);

        }


    }






    public void moveToPosition(int tics, double power){

        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();

        int rLiftPos = -1 * rLift1.getCurrentPosition();
        int lLiftPos = lLift1.getCurrentPosition();


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


                rLiftPos = -1 * rLift1.getCurrentPosition();
                lLiftPos = lLift1.getCurrentPosition();

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


                rLiftPos = -1 * rLift1.getCurrentPosition();
                lLiftPos = lLift1.getCurrentPosition();

                setPower(rightPower, leftPower);


            }
        }

        setPower(0);

    }

    public void moveToPosition(int tics, double power, double kickout){

        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();

        int rLiftPos = -1 * rLift1.getCurrentPosition();
        int lLiftPos = lLift1.getCurrentPosition();


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


                rLiftPos = -1 * rLift1.getCurrentPosition();
                lLiftPos = lLift1.getCurrentPosition();

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


                rLiftPos = -1 * rLift1.getCurrentPosition();
                lLiftPos = lLift1.getCurrentPosition();

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

        int rLiftPos = -1 * rLift1.getCurrentPosition();
        int lLiftPos = lLift1.getCurrentPosition();

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


                rLiftPos = -1 * rLift1.getCurrentPosition();
                lLiftPos = lLift1.getCurrentPosition();

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


                rLiftPos = -1 * rLift1.getCurrentPosition();
                lLiftPos = lLift1.getCurrentPosition();

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


        int rLiftPos = -1 * rLift1.getCurrentPosition();
        int lLiftPos = lLift1.getCurrentPosition();

        int leftError = tics - lLiftPos;
        int rightError = tics - rLiftPos;

        int leftPreviousError = leftError;
        int rightPreviousError = rightError;



        //Dont know if need the != condition
        //if ((tics == 0 && rLiftPos != 0 && lLiftPos != 0)) {

        //TODO: Check if logic for encoder positions works

        while ((time.seconds() - startTime) < 1.25 && Math.abs(leftError) > 2 || Math.abs(leftError) > 2) {

            //TODO: figure out if we need to negate either of them


            rLiftPos = -1 * rLift1.getCurrentPosition();
            lLiftPos = lLift1.getCurrentPosition();

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

    public void linkageAutomated(boolean up){
        if(up){
            setLinkagePosition(constants.linkageUp);
        }else{
            setLinkagePosition(constants.linkageDown);

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
            setLinkagePosition(constants.linkageUp);
        }else{
            setLinkagePosition(constants.linkageDown);
        }
    }

    public int getEncoderPosition(boolean right){
        if(right){
            return rLift1.getCurrentPosition();
        }else{
            return lLift1.getCurrentPosition();

        }
    }

    public void reset(){
        lLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public boolean isBusy(){
        return rLift1.isBusy() && lLift1.isBusy();
    }


    public void newLiftPID(int tics){


        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        double actualStartTime = startTime;



        //TODO: check if we need to negate any

        int rightPos = -1 * getRightEncoderPos();
        int leftPos = getLeftEncoderPos();

        int rightError = tics - rightPos;
        int leftError = tics - leftPos;

        int rightPreviousError = 0;
        int leftPreviousError = 0;

        int rightIntegralSum = 0;
        int leftIntegralSum = 0;

        while(Math.abs(rightError) > 2 && Math.abs(leftError) > 2 && (time.seconds() - actualStartTime) < 1.5){
            //telemetry.addData("target", tics);


            //TODO: check if we need to negate any

            rightPos = -1 * getRightEncoderPos();
            leftPos = getLeftEncoderPos();


            //telemetry.addData("rightPos", rightPos);
            //telemetry.addData("leftPos", leftPos);

            rightError = tics - rightPos;
            leftError = tics - leftPos;

            double currentTime = time.seconds();

            //telemetry.addData("rightError", rightError);
            //telemetry.addData("leftError", leftError);


            rightIntegralSum += (0.5 * (rightError + rightPreviousError) * (currentTime - startTime));
            leftIntegralSum += (0.5 * (leftError + leftPreviousError) * (currentTime - startTime));

            //telemetry.addData("rightIntegralSum", rightIntegralSum);
            //telemetry.addData("leftIntegralSum", leftIntegralSum);



            //TODO: look at telemetry and see if we can have new bound (change integral sum limit)
            if(rightIntegralSum > 20000){
                rightIntegralSum = 20000;
            }else if(rightIntegralSum < -20000){
                rightIntegralSum = -20000;
            }

            if(leftIntegralSum > 20000){
                leftIntegralSum = 20000;
            }else if(leftIntegralSum < -20000){
                leftIntegralSum = -20000;
            }



            double rightDerivative = (rightError - rightPreviousError)/(currentTime - startTime);
            double leftDerivative = (leftError - leftPreviousError)/(currentTime - startTime);

            //telemetry.addData("rightDerivative", rightDerivative);
            //telemetry.addData("leftDerivative", leftDerivative);

            double rightPower = ((pidf.p * rightError) + (pidf.i * rightIntegralSum) + (pidf.d * rightDerivative));
            double leftPower = ((pidf.p * leftError) + (pidf.i * leftIntegralSum) + (pidf.d * leftDerivative));


            //telemetry.addData("rightError", rightError);
            //telemetry.addData("leftError", leftError);

            setPower(rightPower, leftPower);


            startTime = currentTime;
            rightPreviousError = rightError;
            leftPreviousError = leftError;


            //telemetry.update();


        }

        //setPower(0,0,0,0);
        setPower(0);
    }






    public void setGrabberPosition(double position){
        grabber.setPosition(position);
    }

    public double getGrabberPosition(){
        return grabber.getPosition();
    }


    public void grabberOpenAndClose(boolean close){
        if(close) {
            setGrabberPosition(constants.grabbing);
        }else{
            setGrabberPosition(constants.open);
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
