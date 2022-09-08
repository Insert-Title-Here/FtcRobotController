package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecDrive {

    private DcMotorEx fl, fr, bl, br;
    private Robot robot;
    private boolean isDriveOnChub = true;
    private boolean pidEnabled;
    Telemetry telemetry;

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
        CorrectMotors();
    }




    public void moveToPosition(Point p, double velocity){

        double robotDirection = robot.getDirection();
        double direction = Utils.wrapAngle(p.getDirection()) - Utils.wrapAngle(robotDirection);
        int flPosition = (int)(Math.sin(direction) * p.magFromOrigin());
        int frPosition = (int)(Math.cos(direction) * p.magFromOrigin());
        int blPosition = (int)(Math.cos(direction) * p.magFromOrigin());
        int brPosition = (int)(Math.sin(direction) * p.magFromOrigin());

        double flVelocity = Math.max(1, velocity * Math.sin(direction));
        double frVelocity = Math.max(1, velocity * Math.cos(direction));
        double blVelocity = Math.max(1, velocity * Math.cos(direction));
        double brVelocity = Math.max(1, velocity * Math.sin(direction));


        if(pidEnabled){
            setPIDVelocity(flVelocity, flPosition, frVelocity, frPosition, blVelocity, blPosition, brVelocity, brPosition);
        }else {
            LynxModule.BulkData data = robot.getBulkPacket(isDriveOnChub);

            int flPos = data.getMotorCurrentPosition(0);
            int frPos = data.getMotorCurrentPosition(1);
            int blPos = data.getMotorCurrentPosition(2);
            int brPos = data.getMotorCurrentPosition(3);
            while (opModeIsRunning() && flPos < flPosition && frPos < frPosition && blPos < blPosition && brPos < brPosition) {
                setMotorVelocity(flVelocity, frVelocity, blVelocity, brVelocity);

                data = robot.getBulkPacket(isDriveOnChub);

                flPos = data.getMotorCurrentPosition(0);
                frPos = data.getMotorCurrentPosition(1);
                blPos = data.getMotorCurrentPosition(2);
                brPos = data.getMotorCurrentPosition(3);
            }
            brake();
        }
    }

    private void brake() {
        robot.setShouldUpdate(false);
        fl.setVelocity(0);
        fr.setVelocity(0);
        bl.setVelocity(0);
        br.setVelocity(0);
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


    private double P = 0.1; //TODO starting values to be tuned with Zeiger Nichols method
    private double I = 0;
    private double D = 0;
    private double F = 0;

    double integralSumLimit = 0.3;

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

    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(-flPow);
        fr.setPower(frPow) ;
        bl.setPower(-blPow);
        br.setPower(brPow);
    }


    public double setPowerAuto(double power, MecanumDriveTrain.MovementType movement) {
        if(movement == MecanumDriveTrain.MovementType.STRAIGHT) {
            setPower(power, power, power, power);
        }else if(movement == MecanumDriveTrain.MovementType.STRAFE){
            setPower(power, -power, -power, power);
        }else if(movement == MecanumDriveTrain.MovementType.ROTATE){
            setPower(power, -power, power, -power);
        }else if(movement == MecanumDriveTrain.MovementType.LDIAGONAL){
            setPower(power, 0, 0, power);
        }else if(movement == MecanumDriveTrain.MovementType.RDIAGONAL){
            setPower(0, power, power, 0);
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
}
