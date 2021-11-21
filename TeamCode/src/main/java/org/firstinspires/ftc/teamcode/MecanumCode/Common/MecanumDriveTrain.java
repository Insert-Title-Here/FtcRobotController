package org.firstinspires.ftc.teamcode.MecanumCode.Common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDriveTrain {
    private static final double ANGULAR_TOLERANCE = 0.05;
    final double COUNTS_PER_INCH = 920.111004;

    /*
    This has most of the relevant information regarding a 4 wheel Mecanum DriveTrain,
    which is the most used DriveTrain in FTC
     */

    private DcMotor fl, fr, bl, br;



    /**
     * PID Constants
     *
     */
    final double pVelocity = 0.000725; //0.000725
    final double dVelocity  = 0.047; //0.027
    final double pOmega = 0.1;
    final double dOmega = 0;

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



    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(flPow);
        fr.setPower(frPow);
        bl.setPower(blPow);
        br.setPower(brPow);
    }



    private int getSign(double num){
        if(num < 0){
            return -1;
        }else {
            return 1;
        }
    }







}