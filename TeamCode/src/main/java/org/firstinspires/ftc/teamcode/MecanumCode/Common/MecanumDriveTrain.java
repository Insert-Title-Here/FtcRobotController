package org.firstinspires.ftc.teamcode.MecanumCode.Common;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.sql.Wrapper;

public class MecanumDriveTrain {
    private static final double ANGULAR_TOLERANCE = 0.05;
    final double COUNTS_PER_INCH = 920.111004;

    /*
    This has most of the relevant information regarding a 4 wheel Mecanum DriveTrain,
    which is the most used DriveTrain in FTC
     */

    private DcMotor fl, fr, bl, br;

    private double previousVelocity;
    private double[] previousPositions;
    private double previousError;

    /**
     * PID Constants
     *
     */
    final double pVelocity = 0.000725; //0.000725
    final double dVelocity  = 0.047; //0.027

    public MecanumDriveTrain(HardwareMap hardwareMap){
        fl = hardwareMap.dcMotor.get("FrontLeftDrive");
        fr = hardwareMap.dcMotor.get("FrontRightDrive");
        bl = hardwareMap.dcMotor.get("BackLeftDrive");
        br = hardwareMap.dcMotor.get("BackRightDrive");
        correctMotors();

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







    public String getMotorPower(){
        return "fl: " + fl.getPower() + "\n" +
                "fr: " + fr.getPower() + "\n" +
                "bl: " + bl.getPower() + "\n" +
                "br: " + br.getPower();
    }



    private void brake() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }


    public DcMotor[] getMotors(){
        return new DcMotor[]{fl,fr,bl,br};
    }


    /*
    gets the robot driving in a specified direction
     */
    public void setPower(Vector2D velocity, double turnValue){
        turnValue = -turnValue;
        double direction = velocity.getDirection();


        double power = velocity.magnitude();

        double angle = direction + 3*Math.PI / 4.0;
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        setPower((power * sin - turnValue),(power * cos + turnValue),
                (power * cos - turnValue), (power * sin + turnValue));
    }


    /**
     * a method for driving in the cardinal directions of the 3 dimensional space (x,y,theta)
     * @param desiredVelocity velocity in tics / sec the robot should attempt to reach
     * @param tics the distance the robot should travel in tics
     * @param movement the type of movement the robot should perform, straight, strafe, or rotate
     */

    //TODO write a tics to inches conversion as well as a tics to degrees conversion
    public void driveAuto(double desiredVelocity, int tics, MovementType movement){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        OpModeWrapper currentOpMode = OpModeWrapper.currentOpMode();
        double startTime = currentOpMode.time;

        previousVelocity = 0;
        previousPositions = new double[]{0,0,0,0};
        previousError = 0;

        while(isFar(tics) && currentOpMode.opModeIsActive()){
            double currentTime = currentOpMode.time;

            double dt = currentTime - startTime;

            currentOpMode.telemetry.addData("dt", dt);

            double currentVelocity;
            double[] currentPositions = new double[]{fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition()};
            double deltaPositions = computeDeltas(currentPositions);

            if(currentTime - startTime < 0.01){ //TODO not sure this will work the way we want it to, the top may invoke every time causing bad things to happen.
                currentVelocity = 0;
            }else {
                currentVelocity = deltaPositions / dt;
            }
            currentOpMode.telemetry.addData("velocity",currentVelocity);
            currentOpMode.telemetry.update();
            double velocityError = desiredVelocity - currentVelocity;
            double dError = velocityError - previousError;
            double output = velocityError * pVelocity + dError * dVelocity;
            double passedValue = previousVelocity + output;
            if(passedValue > 1.0){
                desiredVelocity = currentVelocity;
                passedValue = 1.0;
            }else if(passedValue < -1.0){
                desiredVelocity = currentVelocity;
                passedValue = -1.0;
            }
            previousVelocity = setPowerAuto(passedValue, movement);
            previousPositions = new double[]{currentPositions[0], currentPositions[1], currentPositions[2], currentPositions[3]};
            previousError = velocityError;

        }
        brake();

    }

    private boolean isFar(int tics){
        return Math.abs(tics - fl.getCurrentPosition()) > 10 || Math.abs(tics - fl.getCurrentPosition()) > 10
                || Math.abs(tics - fl.getCurrentPosition()) > 10 || Math.abs(tics - fl.getCurrentPosition()) > 10;
    }



    public enum MovementType{
        STRAIGHT, STRAFE, ROTATE
    }

    private double computeDeltas(double[] currentPositions){
        double flDelta = Math.abs(currentPositions[0]) - Math.abs(previousPositions[0]);
        double frDelta = Math.abs(currentPositions[1]) - Math.abs(previousPositions[1]);
        double blDelta = Math.abs(currentPositions[2]) - Math.abs(previousPositions[2]);
        double brDelta = Math.abs(currentPositions[3]) - Math.abs(previousPositions[3]);
        return (flDelta + frDelta + blDelta + brDelta) / 4.0;
    }



    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(-flPow);
        fr.setPower(-frPow) ;
        bl.setPower(blPow);
        br.setPower(brPow);
    }

    private double setPowerAuto(double power, MovementType movement){
        if(movement == MovementType.STRAIGHT) {
            setPower(power, power, power, power);
        }else if(movement == MovementType.STRAFE){
            setPower(power, -power, -power, power);
        }else if(movement == MovementType.ROTATE){
            setPower(power, -power, power, -power);
        }
        return power;
    }



    private int getSign(double num){
        if(num < 0){
            return -1;
        }else {
            return 1;
        }
    }







}