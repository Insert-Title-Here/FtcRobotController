package org.firstinspires.ftc.teamcode.Testing.SubsystemsTest.Subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Vector2D;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;


//TODO: Fix getting encoder values (negations)
public class MecDriveBulk {

    private Data data;
    private DcMotorEx fl, fr, bl, br;


    //Original
    PIDCoefficients pidf = new PIDCoefficients(0.006, 0, 0.0003);
    PIDCoefficients rotate = new PIDCoefficients(0.975, 0, 0.02);
    PIDCoefficients rotateFaster = new PIDCoefficients(1.12, 0, 0.01);

    int pStraightVel = 4;
    int pRotateVel = 1700;
    private final int denom = 2;


    File loggingFile = AppUtil.getInstance().getSettingsFile("telemetry.txt");
    String loggingString;

    public enum MovementType {
        STRAIGHT,
        STRAFE,
        ROTATE,
    }


    public MecDriveBulk(HardwareMap hardwareMap, Data data) {
        fl = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        br = hardwareMap.get(DcMotorEx.class, "BackRightDrive");


        //robot.setShouldUpdate(false);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.data = data;

        CorrectMotors();
    }


    public void tankRotate(double radians, double power) {

        radians = wrapAngle(radians);

        data.setUpdate(false);
        double angle = data.getAngle().firstAngle;
        data.setUpdate(true);

        if (radians > angle) {
            power *= -1;
        }

        while (Math.abs(angle - radians) > 0.007) {
            setPowerAuto(power, MecDriveBulk.MovementType.ROTATE);

            data.setUpdate(false);
            angle = data.getAngle().firstAngle;
            data.setUpdate(true);
        }

        brake();
    }


    public void tankRotatePID(double radians, double power, boolean slidesUp, double kickout) {

        data.setUpdate(false);
        double angle = data.getAngle().firstAngle;
        data.setUpdate(true);

        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        double actualStartTime = startTime;

        radians = wrapAngle(radians);
        double radError = wrapAngle(angle - radians);
        double previousError = radError;
        double integralSum = 0;


        while (Math.abs(radError) > 0.005 && (time.seconds() - actualStartTime) < kickout) {


            double newPower = 0;

            data.setUpdate(false);
            angle = data.getAngle().firstAngle;
            data.setUpdate(true);

            double currentTime = time.seconds();

            radError = wrapAngle(angle - radians);


            double derivative = (radError - previousError) / (currentTime - startTime);


            if (!slidesUp) {
                newPower = ((rotate.p * radError) + (rotate.i * integralSum) + (rotate.d * derivative));
            } else {
                newPower = ((rotateFaster.p * radError) + (rotateFaster.i * integralSum) + (rotateFaster.d * derivative));
            }
            setPowerAuto(newPower, MecDriveBulk.MovementType.ROTATE);


            startTime = currentTime;
            previousError = radError;

        }

        brake();


    }

    //TODO: maybe get this working later (if need be)
    public void tankRotatePIDVel(double radians, double kickout) {


        data.setUpdate(false);
        double angle = data.getAngle().firstAngle;
        data.setUpdate(true);

        ElapsedTime time = new ElapsedTime();
        double actualStartTime = time.seconds();

        radians = wrapAngle(radians);
        double radError = wrapAngle(angle - radians);


        while (Math.abs(radError) > 0.005 && (time.seconds() - actualStartTime) < kickout) {


            double newPower = 0;

            data.setUpdate(false);
            angle = data.getAngle().firstAngle;
            data.setUpdate(true);

            radError = wrapAngle(angle - radians);


            newPower = ((pRotateVel * radError) + 10);

            setVelocity(newPower, -newPower, newPower, -newPower);

        }

        brake();

    }


    public void tankRotatePIDSpecial(double radians, double power, boolean slidesUp, double kickout) {

        data.setUpdate(false);
        double angle = data.getAngle().firstAngle;
        data.setUpdate(true);

        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        double actualStartTime = startTime;

        radians = wrapAngle(radians);
        double radError = wrapAngle(angle - radians);
        double previousError = radError;
        double integralSum = 0;


        while (Math.abs(radError) > 0.005 && (time.seconds() - actualStartTime) < kickout) {

            double newPower = 0;

            data.setUpdate(false);
            angle = data.getAngle().firstAngle;
            data.setUpdate(true);

            double currentTime = time.seconds();

            radError = wrapAngle(angle - radians);

            double derivative = (radError - previousError) / (currentTime - startTime);


            if (!slidesUp) {
                newPower = ((rotate.p * radError) + (rotate.i * integralSum) + (rotate.d * derivative));
            } else {
                newPower = ((rotateFaster.p * radError) + (rotateFaster.i * integralSum) + (rotateFaster.d * derivative));
            }


            if (Math.abs(newPower) < 0.15) {
                break;
            }

            setPowerAuto(newPower, MecDriveBulk.MovementType.ROTATE);


            startTime = currentTime;
            previousError = radError;

        }

        power = 0.27;

        data.setUpdate(false);
        angle = data.getAngle().firstAngle;
        data.setUpdate(true);

        if (radians > angle) {
            power *= -1;
        }


        while (Math.abs(angle - radians) > 0.007) {
            data.setUpdate(false);
            angle = data.getAngle().firstAngle;
            data.setUpdate(true);

            setPowerAuto(power, MecDriveBulk.MovementType.ROTATE);
        }

        brake();


    }

    public double wrapAngle(double angle) {
        while (angle > Math.PI) {
            angle -= (2 * Math.PI);
        }

        while (angle < -Math.PI) {
            angle += (2 * Math.PI);
        }

        return angle;
    }


    public void simpleMoveToPosition(int tics, MovementType movement, double power) {

        if (avgPos() > tics) {
            power *= -1;
        }
        while (avgPos() < Math.abs(tics)) {
            setPowerAuto(power, movement);
        }

        brake();

    }

    public double avgPos() {
        data.setUpdate(false);
        LynxModule.BulkData chub = data.chubData;
        data.setUpdate(true);
        return (Math.abs(chub.getMotorCurrentPosition(0)) + Math.abs(chub.getMotorCurrentPosition(1))
                + Math.abs(chub.getMotorCurrentPosition(2)) + Math.abs(chub.getMotorCurrentPosition(3))) / 4;
    }


    public void brake() {
        data.setUpdate(false);
        setPower(0, 0, 0, 0);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        data.setUpdate(true);

    }

    private double calculateAvgStartPower(int tics) {

        tics *= ((denom - 1) / denom);

        data.setUpdate(false);
        LynxModule.BulkData chub = data.getChubData();
        data.setUpdate(true);


        int flPos = -1 * chub.getMotorCurrentPosition(0);
        int frPos = chub.getMotorCurrentPosition(1);
        int blPos = -1 * chub.getMotorCurrentPosition(2);
        int brPos = chub.getMotorCurrentPosition(3);


        int flError = tics - flPos;
        int frError = tics - frPos;
        int blError = tics - blPos;
        int brError = tics - brPos;


        double flPower = ((pidf.p * flError));
        double frPower = ((pidf.p * frError));
        double blPower = ((pidf.p * blError));
        double brPower = ((pidf.p * brError));

        return (flPower + frPower + blPower + brPower) / 4;


    }


    public void goTOPIDPosWithRampUp(int tics, MovementType movement, double limiter) {
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        double actualStartTime = startTime;

        boolean rampUp = true;

        data.setUpdate(false);
        LynxModule.BulkData chub = data.getChubData();
        data.setUpdate(true);


        int flPos = -1 * chub.getMotorCurrentPosition(0);
        int frPos = chub.getMotorCurrentPosition(1);
        int blPos = -1 * chub.getMotorCurrentPosition(2);
        int brPos = chub.getMotorCurrentPosition(3);

        int flError = tics - flPos;
        int frError = tics - frPos;
        int blError = tics - blPos;
        int brError = tics - brPos;

        int flPreviousError = flError;
        int frPreviousError = frError;
        int blPreviousError = blError;
        int brPreviousError = brError;


        int flIntegralSum = 0;
        int frIntegralSum = 0;
        int blIntegralSum = 0;
        int brIntegralSum = 0;


        while (Math.abs(flError) > 2 && Math.abs(frError) > 2 && Math.abs(blError) > 2 && Math.abs(brError) > 2 && (time.seconds() - actualStartTime) < 1.5) {

            //I dont think the ramp up part actually works
            if (rampUp) {

                double start = calculateAvgStartPower(tics);
                int targetRamp = tics / denom;

                data.setUpdate(false);
                chub = data.getChubData();
                data.setUpdate(true);

                flPos = -1 * chub.getMotorCurrentPosition(0);
                frPos = chub.getMotorCurrentPosition(1);
                blPos = -1 * chub.getMotorCurrentPosition(2);
                brPos = chub.getMotorCurrentPosition(3);


                while ((flPos + frPos + blPos + brPos) / 4 < targetRamp) {

                    double rampPower = ((denom - 1) * pidf.p) * ((frPos + flPos + brPos + blPos) / 4);

                    if (Math.abs(rampPower) > limiter) {
                        if (rampPower < 0) {
                            rampPower = -1 * limiter;
                        } else {
                            rampPower = limiter;
                        }
                    }

                    setPower(rampPower, rampPower, rampPower, rampPower);


                    data.setUpdate(false);
                    chub = data.getChubData();
                    data.setUpdate(true);

                    flPos = -1 * chub.getMotorCurrentPosition(0);
                    frPos = chub.getMotorCurrentPosition(1);
                    blPos = -1 * chub.getMotorCurrentPosition(2);
                    brPos = chub.getMotorCurrentPosition(3);
                }
                rampUp = false;

            } else {


                data.setUpdate(false);
                chub = data.getChubData();
                data.setUpdate(true);

                flPos = -1 * chub.getMotorCurrentPosition(0);
                frPos = chub.getMotorCurrentPosition(1);
                blPos = -1 * chub.getMotorCurrentPosition(2);
                brPos = chub.getMotorCurrentPosition(3);


                flError = tics - flPos;
                frError = tics - frPos;
                blError = tics - blPos;
                brError = tics - brPos;


                double currentTime = time.seconds();


                double flDerivative = (flError - flPreviousError) / (currentTime - startTime);
                double frDerivative = (frError - frPreviousError) / (currentTime - startTime);
                double blDerivative = (blError - blPreviousError) / (currentTime - startTime);
                double brDerivative = (brError - brPreviousError) / (currentTime - startTime);


                double flPower = ((pidf.p * flError) + (pidf.i * flIntegralSum) + (pidf.d * flDerivative));
                double frPower = ((pidf.p * frError) + (pidf.i * frIntegralSum) + (pidf.d * frDerivative));
                double blPower = ((pidf.p * blError) + (pidf.i * blIntegralSum) + (pidf.d * blDerivative));
                double brPower = ((pidf.p * brError) + (pidf.i * brIntegralSum) + (pidf.d * brDerivative));

                if (Math.abs(flPower) > limiter) {
                    if (flPower < 0) {
                        flPower = -1 * limiter;
                    } else {
                        flPower = limiter;
                    }
                }

                if (Math.abs(frPower) > limiter) {
                    if (frPower < 0) {
                        frPower = -1 * limiter;
                    } else {
                        frPower = limiter;
                    }
                }

                if (Math.abs(blPower) > limiter) {
                    if (blPower < 0) {
                        blPower = -1 * limiter;
                    } else {
                        blPower = limiter;
                    }
                }

                if (Math.abs(brPower) > limiter) {
                    if (brPower < 0) {
                        brPower = -1 * limiter;
                    } else {
                        brPower = limiter;
                    }
                }


                setPower(flPower, frPower, blPower, brPower);

                startTime = currentTime;
                flPreviousError = flError;
                frPreviousError = frError;
                blPreviousError = blError;
                brPreviousError = brError;

            }


        }

        brake();

    }

    public void pidVelWithRampUp(int tics, int millis) {
        ElapsedTime time = new ElapsedTime();
        double actualStartTime = time.seconds();


        data.setUpdate(false);
        LynxModule.BulkData chub = data.getChubData();
        data.setUpdate(true);

        int flPos = -1 * chub.getMotorCurrentPosition(0);
        int frPos = chub.getMotorCurrentPosition(1);
        int blPos = -1 * chub.getMotorCurrentPosition(2);
        int brPos = chub.getMotorCurrentPosition(3);

        int flError = tics - flPos;
        int frError = tics - frPos;
        int blError = tics - blPos;
        int brError = tics - brPos;


        rampUpVelocity(millis, tics, time);

        while (Math.abs(flError) > 2 && Math.abs(frError) > 2 && Math.abs(blError) > 2 && Math.abs(brError) > 2 && (time.seconds() - actualStartTime) < 1.5) {


            data.setUpdate(false);
            chub = data.getChubData();
            data.setUpdate(true);


            flPos = -1 * chub.getMotorCurrentPosition(0);
            frPos = chub.getMotorCurrentPosition(1);
            blPos = -1 * chub.getMotorCurrentPosition(2);
            brPos = chub.getMotorCurrentPosition(3);


            flError = tics - flPos;
            frError = tics - frPos;
            blError = tics - blPos;
            brError = tics - brPos;


            double flPower = (pStraightVel * flError) + 10;
            double frPower = (pStraightVel * frError) + 10;
            double blPower = (pStraightVel * blError) + 10;
            double brPower = (pStraightVel * brError) + 10;


            setPower(flPower, frPower, blPower, brPower);


        }

        brake();

    }

    private void rampUpVelocity(int millis, int targetTics, ElapsedTime time) {
        double startTime = time.milliseconds();
        double slope = (pStraightVel * targetTics) / millis;
        while (time.milliseconds() - startTime < millis) {
            setVelocity(slope * (time.milliseconds() - startTime), slope * (time.milliseconds() - startTime), slope * (time.milliseconds() - startTime), slope * (time.milliseconds() - startTime));
        }

        setVelocity(pStraightVel * targetTics, pStraightVel * targetTics, pStraightVel * targetTics, pStraightVel * targetTics);
    }


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

    public void fieldCentricSetPower(Vector2D velocity, double turnValue, boolean isSwapped) {

        data.setUpdate(false);
        double currentAngle = data.getAngle().firstAngle;
        data.setUpdate(true);

        turnValue = -turnValue;
        double direction = velocity.getDirection() - currentAngle;


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


    public double setPowerAuto(double power, MecDriveBulk.MovementType movement) {
        if (movement == MecDriveBulk.MovementType.STRAIGHT) {
            setPower(power, power, power, power);
        } else if (movement == MecDriveBulk.MovementType.STRAFE) {
            setPower(power, -power, -power, power);
        } else if (movement == MecDriveBulk.MovementType.ROTATE) {
            setPower(power, -power, power, -power);
        }
        return power;
    }


    public void addToLoggingString(String add) {
        loggingString += (add + "\n");
    }

    public void writeLoggerToFile() {
        try {
            PrintStream toFile = new PrintStream(loggingFile);
            toFile.println(loggingString);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }
}