package org.firstinspires.ftc.teamcode.Testing.SubsystemsTest.Subsystems;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;


import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

//TODO: Check lift ports and negations
public class ScoringSystemBulk {

    Data data;
    public DcMotorEx lLift1, rLift1, lLift2, rLift2;
    public Servo grabber;
    ServoImplEx rLinkage, lLinkage;
    public ScoringMode height;
    private int coneStack, rightPreviousError, leftPreviousError, liftTarget;
    private double startTime, currentTime;
    ElapsedTime time;

    private boolean manualFlag, autoLinkageFlag, extended, changeStackFlag, liftBrokenMode, changeToggle, toLinkageDown, toLinkageUp;

    PIDCoefficients pidf = new PIDCoefficients(0.0085, 0.0000275, 0.00023);

    File file = AppUtil.getInstance().getSettingsFile("motion.txt");
    String composite = "";




    public enum ScoringMode {
        HIGH,
        MEDIUM,
        LOW,
        ULTRA
    }

    public ScoringSystemBulk(HardwareMap hardwareMap, Data data, boolean up) {

        this.data = data;
        time = new ElapsedTime();

        coneStack = 1;
        height = ScoringMode.HIGH;
        extended = false;

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


        if (up) {
            setLinkagePosition(Constants.linkageUpV2);
        } else {
            setLinkagePosition(Constants.linkageDownV2);
        }

        grabber.setPosition(Constants.open);

    }


    public int getConeStack() {
        return coneStack;
    }

    public void lowerConeStack() {
        if (coneStack - 1 > 0) {
            coneStack -= 1;
        }
    }

    public void raiseConeStack() {
        if (coneStack + 1 < 6) {
            coneStack += 1;
        }
    }

    public void setConeStack(int height) {
        coneStack = height;

    }

    public int getLiftTarget() {
        return liftTarget;
    }

    public boolean getAutoLinkageFlag() {
        return autoLinkageFlag;
    }

    public boolean getChangeStackFlag() {
        return changeStackFlag;
    }

    public void setChangeStackFlag(boolean changeStackFlag) {
        this.changeStackFlag = changeStackFlag;
    }

    public void setAutoLinkageFlag(boolean autoLinkageFlag) {
        this.autoLinkageFlag = autoLinkageFlag;
    }


    //TODO: tune this
    public void setLinkageConeStack(boolean logistic) {
        if (logistic) {
            if (coneStack == 5) {
                setLinkagePositionLogistic(0.2525, 300);
            } else if (coneStack == 4) {
                setLinkagePositionLogistic(0.2115, 300);
            } else if (coneStack == 3) {
                setLinkagePositionLogistic(0.1785, 300);
            } else if (coneStack == 2) {
                setLinkagePositionLogistic(Constants.linkageDownV2, 300);
            } else if (coneStack == 1) {
                setLinkagePositionLogistic(Constants.linkageDownV2, 300);
            }
        } else {
            if (coneStack == 5) {
                setLinkagePosition(0.2525);
            } else if (coneStack == 4) {
                setLinkagePosition(0.2115);
            } else if (coneStack == 3) {
                setLinkagePosition(0.1785);
            } else if (coneStack == 2) {
                setLinkagePosition(Constants.linkageDownV2);
            } else if (coneStack == 1) {
                setLinkagePosition(Constants.linkageDownV2);
            }
        }
    }

    public int getRightEncoderPos() {
        return rLift1.getCurrentPosition();
    }


    public boolean isExtended() {
        return extended;
    }

    public void setExtended(boolean extended) {
        this.extended = extended;
    }

    public void setScoringMode(ScoringMode height) {
        this.height = height;
    }

    public ScoringMode getScoringMode() {
        return height;
    }

    public int getHeight() {
        if (height == ScoringMode.HIGH) {
            return 1350;
        } else if (height == ScoringMode.MEDIUM) {
            return 750;
        } else if (height == ScoringMode.LOW) {
            return 250;
        }

        return 0;
    }

    public void setPower(double power) {
        rLift1.setPower(-power);
        lLift1.setPower(power);

        rLift2.setPower(-power);
        lLift2.setPower(power);
    }

    public void setPower(double rightPower, double leftPower) {
        rLift1.setPower(-rightPower);
        lLift1.setPower(leftPower);

        rLift2.setPower(-rightPower);
        lLift2.setPower(leftPower);
    }

    public void setVelocity(double rightPower, double leftPower) {
        rLift1.setVelocity(-rightPower);
        lLift1.setVelocity(leftPower);

        rLift2.setVelocity(-rightPower);
        lLift2.setVelocity(leftPower);
    }


    public void changeMode(ScoringMode score) {
        height = score;
    }

    public void autoGoToPosition() {


        if (height == ScoringMode.HIGH /*|| height == ScoringMode.ULTRA*/) {
            if (getConeStack() > 1) {
                moveToPosition(885, 1, 1);
            } else {
                moveToPosition(960, 1, 1.2);
            }

        } else if (height == ScoringMode.MEDIUM) {
            moveToPosition(500, 1, 0.8);


        } else if (height == ScoringMode.LOW) {
            moveToPosition(80, 1, 0.5);

        }

        extended = true;
    }

    public void commandAutoGoToPosition() {
        if (height == ScoringMode.HIGH /*|| height == ScoringMode.ULTRA*/) {
            setLiftTarget(1025);

        } else if (height == ScoringMode.MEDIUM) {
            setLiftTarget(610);


        } else if (height == ScoringMode.LOW) {
            setLiftTarget(165);

        }

        extended = true;
    }


    public void moveToPosition(int tics, double power, double kickout) {

        double startTime = time.seconds();

        data.setUpdate(false);
        LynxModule.BulkData ehub = data.getEhubData();
        data.setUpdate(true);


        int rLiftPos = -1 * ehub.getMotorCurrentPosition(2);
        int lLiftPos = -1 * ehub.getMotorCurrentPosition(3);


        if (tics < ((rLiftPos + lLiftPos) / 2)) {
            power *= -1;
        }

        double rightPower = power;
        double leftPower = power;


        if (power > 0) {
            while ((time.seconds() - startTime) < kickout && (rLiftPos < tics || lLiftPos < tics)) {


                if (rLiftPos >= tics) {
                    rightPower = 0;
                }
                if (lLiftPos >= tics) {
                    leftPower = 0;
                }


                data.setUpdate(false);
                ehub = data.getEhubData();
                data.setUpdate(true);


                rLiftPos = -1 * ehub.getMotorCurrentPosition(2);
                lLiftPos = -1 * ehub.getMotorCurrentPosition(3);

                setPower(rightPower, leftPower);


            }
        } else {
            while ((time.seconds() - startTime) < kickout && (rLiftPos > tics || lLiftPos > tics)) {

                //TODO: figure out if we need to negate either of them

                if (rLiftPos <= tics) {
                    rightPower = 0;
                }

                if (lLiftPos <= tics) {
                    leftPower = 0;
                }


                data.setUpdate(false);
                ehub = data.getEhubData();
                data.setUpdate(true);


                rLiftPos = -1 * ehub.getMotorCurrentPosition(2);
                lLiftPos = -1 * ehub.getMotorCurrentPosition(3);

                setPower(rightPower, leftPower);


            }
        }

        setPower(0);

    }


    public void moveToPIDPosition(int tics, double power) {


        double startTime = time.seconds();
        double intStartTime = time.milliseconds();


        int rightIntegralSum = 0;
        int leftIntegralSum = 0;


        data.setUpdate(false);
        LynxModule.BulkData ehub = data.getEhubData();
        data.setUpdate(true);


        int rLiftPos = -1 * ehub.getMotorCurrentPosition(2);
        int lLiftPos = -1 * ehub.getMotorCurrentPosition(3);

        int leftError = tics - lLiftPos;
        int rightError = tics - rLiftPos;

        int leftPreviousError = leftError;
        int rightPreviousError = rightError;


        //Dont know if need the != condition
        //if ((tics == 0 && rLiftPos != 0 && lLiftPos != 0)) {

        //TODO: Check if logic for encoder positions works

        while ((time.seconds() - startTime) < 1.25 && Math.abs(leftError) > 2 || Math.abs(leftError) > 2) {

            //TODO: figure out if we need to negate either of them


            data.setUpdate(false);
            ehub = data.getEhubData();
            data.setUpdate(true);

            rLiftPos = -1 * ehub.getMotorCurrentPosition(2);
            lLiftPos = -1 * ehub.getMotorCurrentPosition(3);


            double currentTime = time.milliseconds();


            leftError = tics - lLiftPos;
            rightError = tics - rLiftPos;


            leftIntegralSum += (0.5 * (leftError + leftPreviousError) * (currentTime - intStartTime));
            rightIntegralSum += (0.5 * (rightError + rightPreviousError) * (currentTime - intStartTime));

            if (leftIntegralSum > 20000) {
                leftIntegralSum = 20000;
            } else if (leftIntegralSum < -20000) {
                leftIntegralSum = -20000;
            }

            if (rightIntegralSum > 20000) {
                rightIntegralSum = 20000;
            } else if (rightIntegralSum < -20000) {
                rightIntegralSum = -20000;
            }

            double leftDerivative = (leftError - leftPreviousError) / (currentTime - intStartTime);
            double rightDerivative = (rightError - rightPreviousError) / (currentTime - intStartTime);


            double leftPower = ((leftError * pidf.p) + (leftIntegralSum * pidf.i) + (leftDerivative * pidf.d));
            double rightPower = ((rightError * pidf.p) + (rightIntegralSum * pidf.i) + (rightDerivative * pidf.d));


            setPower(rightPower, leftPower);


            intStartTime = currentTime;
            leftPreviousError = leftError;
            rightPreviousError = rightError;


        }

        setPower(0);


    }


    public void setLinkagePosition(double position) {
        //TODO: tune position values
        rLinkage.setPosition(position);
        lLinkage.setPosition(position);
    }

    public void setLinkagePositionLogistic(double target, int sleepTime) {

        int resolution = 100;
        double step = 4.0 / resolution;
        double start = getLeftLinkage();
        double startX = -2.0;

        double startTime = time.milliseconds();


        for (int i = 0; i < resolution; i++) {
            setLinkagePosition(logistic(startX, start, target));
            startX += step;
            sleep(sleepTime / resolution);

            if (time.milliseconds() - startTime > (3 * sleepTime)) {
                break;
            }
        }
        setLinkagePosition(target);
    }

    public void setLinkagePositionLogistic(double target, int sleepTime, int resolution) {
        double step = 4.0 / resolution;
        double start = getLeftLinkage();
        double startX = -2.0;
        double startLoopTime;
        double loopTime;
        for (int i = 0; i < resolution; i++) {
            startLoopTime = time.milliseconds();
            setLinkagePosition(logistic(startX, start, target));
            startX += step;
            loopTime = time.milliseconds() - startLoopTime;
            if ((double) sleepTime / resolution > loopTime) {
                sleep((long) (sleepTime / resolution - loopTime));
            } else {
                if (time.milliseconds() > sleepTime) {
                    break;
                }
            }
        }
        setLinkagePosition(target);
    }

    public double getRightLinkage() {
        return rLinkage.getPosition();
    }

    public double getLeftLinkage() {
        return lLinkage.getPosition();
    }

    public void linkageAutomated(boolean up) {
        if (up) {
            setLinkagePosition(Constants.linkageUp);
        } else {
            setLinkagePosition(Constants.linkageDown);

        }
    }

    //TODO: Tune this
    public void shiftLinkagePosition() {
        double setHeight = getGrabberPosition() - 0.04;

        if (setHeight < 0.09) {
            setHeight = 0.25;
        } else if (setHeight > 0.25) {
            setHeight = 0.25;
        }

        setLinkagePosition(setHeight);
    }


    public void reset() {
        lLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public boolean isBusy() {
        return rLift1.isBusy() && lLift1.isBusy();
    }


    public void newLiftPID(int tics, double limiter, double kickout) {


        double startTime = time.seconds();
        double actualStartTime = startTime;


        data.setUpdate(false);
        LynxModule.BulkData ehub = data.getEhubData();
        data.setUpdate(true);


        int rightPos = -1 * ehub.getMotorCurrentPosition(2);
        int leftPos = -1 * ehub.getMotorCurrentPosition(3);

        int rightError = tics - rightPos;
        int leftError = tics - leftPos;

        int rightPreviousError = 0;
        int leftPreviousError = 0;

        while (Math.abs(rightError) > 2 && Math.abs(leftError) > 2 && (time.seconds() - actualStartTime) < kickout) {


            //TODO: check if we need to negate any

            data.setUpdate(false);
            ehub = data.getEhubData();
            data.setUpdate(true);


            rightPos = -1 * ehub.getMotorCurrentPosition(2);
            leftPos = -1 * ehub.getMotorCurrentPosition(3);


            rightError = tics - rightPos;
            leftError = tics - leftPos;

            double currentTime = time.seconds();


            double rightDerivative = (rightError - rightPreviousError) / (currentTime - startTime);
            double leftDerivative = (leftError - leftPreviousError) / (currentTime - startTime);


            double rightPower = ((pidf.p * rightError) + (pidf.d * rightDerivative));
            double leftPower = ((pidf.p * leftError) + (pidf.d * leftDerivative));

            if (Math.abs(rightPower) > limiter) {
                if (rightPower < 0) {
                    rightPower = -1 * limiter;
                } else {
                    rightPower = limiter;
                }
            }

            if (Math.abs(leftPower) > limiter) {
                if (leftPower < 0) {
                    leftPower = -1 * limiter;
                } else {
                    leftPower = limiter;
                }
            }


            setPower(rightPower, leftPower);


            startTime = currentTime;
            rightPreviousError = rightError;
            leftPreviousError = leftError;


        }

        setPower(0);

    }


    public void newLiftPIDUpdate(double limiter) {
        currentTime = time.seconds();

        data.setUpdate(false);
        LynxModule.BulkData ehub = data.getEhubData();
        data.setUpdate(true);


        int rightPos = -1 * ehub.getMotorCurrentPosition(2);
        int leftPos = -1 * ehub.getMotorCurrentPosition(3);


        int rightError = liftTarget - rightPos;
        int leftError = liftTarget - leftPos;


        double rightDerivative = (rightError - rightPreviousError) / (currentTime - startTime);
        double leftDerivative = (leftError - leftPreviousError) / (currentTime - startTime);


        double rightPower = ((pidf.p * rightError) + (pidf.d * rightDerivative));
        double leftPower = ((pidf.p * leftError) + (pidf.d * leftDerivative));

        if (Math.abs(rightPower) > limiter) {
            if (rightPower < 0) {
                rightPower = -1 * limiter;
            } else {
                rightPower = limiter;
            }
        }

        if (Math.abs(leftPower) > limiter) {
            if (leftPower < 0) {
                leftPower = -1 * limiter;
            } else {
                leftPower = limiter;
            }
        }


        setPower(rightPower, leftPower);


        startTime = currentTime;
        rightPreviousError = rightError;
        leftPreviousError = leftError;

    }

    public void setLiftTarget(int target) {
        liftTarget = target;
    }


    public void setGrabberPosition(double position) {
        grabber.setPosition(position);
    }

    public double getGrabberPosition() {
        return grabber.getPosition();
    }


    public static double logistic(double x, double lower, double upper) {
        double k = 2;
        double x0 = 0;
        upper -= lower;
        return (upper / (1 + Math.pow(Math.E, -k * (x - x0)))) + lower;
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void setManualFlag(boolean manualFlag) {
        this.manualFlag = manualFlag;
    }


    public boolean getManualFlag() {
        return manualFlag;
    }

    public void setLiftBrokenMode(boolean liftBrokenMode) {
        this.liftBrokenMode = liftBrokenMode;
    }


    public boolean getLiftBrokenMode() {
        return liftBrokenMode;
    }

    public void setChangeToggle(boolean changeToggle) {
        this.changeToggle = changeToggle;
    }


    public boolean getChangeToggle() {
        return changeToggle;
    }

    public void setToLinkageDown(boolean toLinkageDown) {
        this.toLinkageDown = toLinkageDown;
    }


    public boolean getToLinkageDown() {
        return toLinkageDown;
    }

    public void setToLinkageUp(boolean toLinkageUp) {
        this.toLinkageUp = toLinkageUp;
    }


    public boolean getToLinkageUp() {
        return toLinkageUp;
    }


    public void addToLoggingString(String add) {
        composite += (add + "\n");
    }

    public void writeLoggerToFile() {
        try {
            PrintStream toFile = new PrintStream(file);
            toFile.print(composite);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

}
