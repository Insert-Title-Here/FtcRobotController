package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecDrive {

    private DcMotorEx fl, fr, bl, br;
    private Robot robot;
    private boolean isDriveOnChub = true;

    /*
    localizer -> Arm/actuator -> drive
     */

    public MecDrive(HardwareMap hardwareMap, Robot robot){
        fl = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        fl = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        fl = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        fl = hardwareMap.get(DcMotorEx.class, "BackRightDrive");
        this.robot = robot;
        CorrectMotors();
    }

    /*
    todo
    fix the 0 direction assumption
    reset the motors somehow
    PID Impl
     */


    public void moveToPosition(Point p, double velocity){
        double direction = p.getDirection();
        int flPosition = (int)(Math.sin(direction) * p.magFromOrigin());
        int frPosition = (int)(Math.cos(direction) * p.magFromOrigin());
        int blPosition = (int)(Math.cos(direction) * p.magFromOrigin());
        int brPosition = (int)(Math.sin(direction) * p.magFromOrigin());

        double flVelocity = Math.max(1, velocity * Math.sin(direction));
        double frVelocity = Math.max(1, velocity * Math.cos(direction));
        double blVelocity = Math.max(1, velocity * Math.cos(direction));
        double brVelocity = Math.max(1, velocity * Math.sin(direction));

        LynxModule.BulkData data = robot.getBulkPacket(isDriveOnChub);

        int flPos = data.getMotorCurrentPosition(0);
        int frPos = data.getMotorCurrentPosition(1);
        int blPos = data.getMotorCurrentPosition(2);
        int brPos = data.getMotorCurrentPosition(3);
        while(opModeIsRunning() && flPos < flPosition && frPos < frPosition && blPos < blPosition && brPos < brPosition){
            setMotorVelocity(flVelocity, frVelocity, blVelocity, brVelocity);

            data = robot.getBulkPacket(isDriveOnChub);

            flPos = data.getMotorCurrentPosition(0);
            frPos = data.getMotorCurrentPosition(1);
            blPos = data.getMotorCurrentPosition(2);
            brPos = data.getMotorCurrentPosition(3);
        }
        brake();
    }

    private void brake() {
        fl.setVelocity(0);
        fr.setVelocity(0);
        bl.setVelocity(0);
        br.setVelocity(0);
    }

    private void setMotorVelocity(double flV, double frV, double blV, double brV){
        fl.setVelocity(flV);
        fr.setVelocity(frV);
        bl.setVelocity(blV);
        br.setVelocity(brV);
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
}
