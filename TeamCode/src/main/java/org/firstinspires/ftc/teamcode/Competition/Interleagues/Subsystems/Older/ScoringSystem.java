package org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Older;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.OpModeWrapper;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Robot;

import java.util.concurrent.atomic.AtomicBoolean;

public class ScoringSystem extends SubsystemBase {

    public enum ExtensionHeight{
        HIGH,
        MEDIUM,
        ZERO
    }

    public DcMotorEx rLift, lLift;
    public Servo grabber, rLinkage, lLinkage; //dont know whether there are two linkage servos or one
    private Robot robot;


    Thread passiveLift;
    AtomicBoolean passiveOn;

    //Constants constants;
    boolean fullyExtended = false;
    boolean isArmOnChub = false;
    boolean pidEnabled;



    public ScoringSystem(HardwareMap hardwareMap, Robot robot, /*//Constants constants,*/ boolean pid){
        this.robot = robot;
        rLift = hardwareMap.get(DcMotorEx.class, "RightLift");
        lLift = hardwareMap.get(DcMotorEx.class, "LeftLift");


        robot.setShouldUpdate(false);

        rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //TODO: Initialize Servos to correct position

        robot.setShouldUpdate(true);

        pidEnabled = pid;

        passiveOn = new AtomicBoolean();
        passiveOn.set(false);

        passiveLift = new Thread(){
            @Override
            public void run() {

                if(passiveOn.get() == true) {
                    passivePID();
                }
            }
        };



        correctMotors();


        lLinkage = hardwareMap.get(Servo.class, "LeftLinkage");
        rLinkage = hardwareMap.get(Servo.class, "RightLinkage");

        grabber = hardwareMap.get(Servo.class, "Grabber");

    }

    public ScoringSystem(HardwareMap hardwareMap, /*//Constants constants,*/ boolean pid){
        //rLift = hardwareMap.get(DcMotorEx.class, "RightLift");
        //lLift = hardwareMap.get(DcMotorEx.class, "LeftLift");

        //rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //TODO: Initialize Servos to correct position


        pidEnabled = pid;




        //correctMotors();


        lLinkage = hardwareMap.get(Servo.class, "LeftLinkage");
        rLinkage = hardwareMap.get(Servo.class, "RightLinkage");

        grabber = hardwareMap.get(Servo.class, "Grabber");


        lLinkage.setPosition(0.03);
        rLinkage.setPosition(0.03);
        grabber.setPosition(Constants.open);


    }

    public void resetAndRunWithoutEncoders(){
        rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    //TODO: change to velocity later if possible and add PID implementation
    public void moveToPosition(int tics, double power){

        if(pidEnabled) {
            setPIDPower(tics);

        }else{

            LynxModule.BulkData data = robot.getBulkPacket(isArmOnChub);

            //TODO: figure out if we need to negate either of them
            int rLiftPos = data.getMotorCurrentPosition(0);
            int lLiftPos = data.getMotorCurrentPosition(1) * -1;



            //Dont know if need the != condition
            if (!fullyExtended || (tics == 0 && rLiftPos != 0 && lLiftPos != 0)) {

                //TODO: Check if logic for encoder positions works
                while (opModeIsRunning() && Math.abs(rLiftPos - tics) > 20 && Math.abs(lLiftPos - tics) > 20) {
                    data = robot.getBulkPacket(isArmOnChub);

                    //TODO: figure out if we need to negate either of them
                    rLiftPos = data.getMotorCurrentPosition(0);
                    lLiftPos = data.getMotorCurrentPosition(1) * -1;

                    setPower(power);


                }

                brake();
            }

        }



    }


    public void moveToPositionNew(int tics, double power){

        int rLiftPos = rLift.getCurrentPosition();
        //int lLiftPos = lLift.getCurrentPosition() * -1;

        if(tics < /*(*/rLiftPos/* + lLiftPos) / 2*/){
            power *= -1;
        }




        //Dont know if need the != condition
        //if ((tics == 0 && rLiftPos != 0 && lLiftPos != 0)) {

        //TODO: Check if logic for encoder positions works
        while (/*opModeIsRunning() && */Math.abs(rLiftPos - tics) > 20 /*&& Math.abs(lLiftPos - tics) > 20*/) {

            //TODO: figure out if we need to negate either of them
            rLiftPos = rLift.getCurrentPosition();
            //lLiftPos = lLift.getCurrentPosition() * -1;

            setPower(power);




        }

        setPower(0);
        //}

    }

    PIDCoefficients pid = new PIDCoefficients(0,0,0);

    //TODO: Figure out what to do with this
    private void passivePID(){
        LynxModule.BulkData data = robot.getBulkPacket(isArmOnChub);

        //Need to read these values once and right after the movement is done
        int leftTics = data.getMotorCurrentPosition(0);
        int rightTics = data.getMotorCurrentPosition(1);



        //while(opModeIsRunning() && Math.abs(rLiftPos - rightTics) > 20 && Math.abs(lLiftPos - leftTics) > 20)

    }




    private void setPIDPower(int tics){
        passiveOn.set(false);

        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        LynxModule.BulkData data = robot.getBulkPacket(isArmOnChub);

        //TODO: figure out if we need to negate either of them
        int rLiftPos = data.getMotorCurrentPosition(0);
        int lLiftPos = data.getMotorCurrentPosition(1);


        int leftPreviousError = Math.abs(tics - rLiftPos);
        int rightPreviousError = Math.abs(tics - lLiftPos);

        int leftIntegralSum = 0;
        int rightIntegralSum = 0;


        //Dont know if need the != condition
        if (!fullyExtended || (tics == 0 && rLiftPos != 0 && lLiftPos != 0)) {

            //TODO: Check if logic for encoder positions works
            while (opModeIsRunning() && Math.abs(rLiftPos - tics) > 20 && Math.abs(lLiftPos - tics) > 20) {
                data = robot.getBulkPacket(isArmOnChub);

                //TODO: figure out if we need to negate either of them
                rLiftPos = data.getMotorCurrentPosition(0);
                lLiftPos = data.getMotorCurrentPosition(1);

                double currentTime = time.seconds();

                int leftError = tics - lLiftPos;
                int rightError = tics - rLiftPos;

                leftIntegralSum += (0.5 * (leftError + leftPreviousError) * (currentTime - startTime));
                rightIntegralSum += (0.5 * (rightError + rightPreviousError) * (currentTime - startTime));

                double leftDerivative = (leftError - leftPreviousError)/(currentTime - startTime);
                double rightDerivative = (rightError - rightPreviousError)/(currentTime - startTime);

                double leftPower = (pid.p * leftError) + (pid.i * leftIntegralSum) + (pid.d * leftDerivative);
                double rightPower = (pid.p * rightError) + (pid.i * rightIntegralSum) + (pid.d * rightDerivative);

                //TODO: check if I need to use this
                /*if(){
                    leftPower *= -1;
                }else if(){
                    rightPower *= -1;
                }
                */

                setPower(leftPower, rightPower);

                startTime = currentTime;
                leftPreviousError = leftError;
                rightPreviousError = rightError;


            }

            brake();

            passiveOn.set(true);
        }

    }

    private boolean opModeIsRunning(){
        return OpModeWrapper.currentOpMode().opModeIsActive() && !OpModeWrapper.currentOpMode().isStopRequested();
    }

    public void brake(){
        robot.setShouldUpdate(false);
        setPower(0);
        rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.setShouldUpdate(true);
    }

    public void correctMotors(){
        rLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setPower(double power){
        rLift.setPower(power);
        lLift.setPower(-power);
    }

    public void setrLiftPower(double power){
        rLift.setPower(power);
    }

    public void setlLiftPower(double power){
        lLift.setPower(power);
    }

    private void setPower(double leftPower, double rightPower){
        lLift.setPower(leftPower);
        rLift.setPower(rightPower);
    }

    private void setVelocity(int velocity){
        //rLift.setVelocity(velocity);
        //lLift.setVelocity(velocity);
    }

    public void setLinkagePosition(double position){
        //TODO: tune position values
        rLinkage.setPosition(position);
        lLinkage.setPosition(position);
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
}
