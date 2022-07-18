package org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubMotor;

public class TalonsArmSystem {

    //private DcMotor lift, ramp, liftEncoder;
    public ExpansionHubMotor liftEncoder, lift, ramp;
    private Servo house, linkage, stop;
    private boolean extended;

    public TalonsArmSystem(HardwareMap hardwareMap){
        lift = (ExpansionHubMotor) hardwareMap.dcMotor.get("lift");
        house = hardwareMap.get(Servo.class, "house");
        linkage = hardwareMap.get(Servo.class, "linkage");
        stop = hardwareMap.get(Servo.class, "stop");
        ramp = (ExpansionHubMotor) hardwareMap.dcMotor.get("ramp");
        liftEncoder = (ExpansionHubMotor) hardwareMap.dcMotor.get("liftEncoder");

        liftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);


        extended = false;

        open();
        down();
        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO: See if we need to set direction
    }

    public void setPower(double power){
        lift.setPower(power);
    }

    public int getPosition(boolean isEncoder){
        if(isEncoder){
            return liftEncoder.getCurrentPosition();
        }else{
            return lift.getCurrentPosition();
        }

    }

    public void runToPosition(int target, double power){

        if(Math.abs(liftEncoder.getCurrentPosition()) > target){
            power = -power;
        }

        while(Math.abs(liftEncoder.getCurrentPosition()) < target){
            setPower(power);

        }

        setPower(0);

    }

    public void extend(double power){
        //TODO: tune extension tic value later
        runToPosition(5000, power);
        //14800
        extended = true;
    }

    public boolean isExtended(){
        return extended;
    }

    public void retract(double power){
        runToPosition(1000, power);
        extended = false;
        liftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void brake(){
        setPower(0);
    }

    public void houseSetPosition(double position){
        house.setPosition(position);
    }

    public void close(){
        //TODO: tune this value
        houseSetPosition(0.2);
    }

    public void score(){
        //TODO: tune this value
        houseSetPosition(0.5);
    }

    public void open(){
        //TODO: tune this value
        houseSetPosition(0.1);
    }

    public void linkageSetPosition(double position){
        linkage.setPosition(position);
    }

    public void down(){
        linkageSetPosition(0.3);
    }
    public void up(){
        linkageSetPosition(0.58);
    }

    public void setRampPower(double power){
        ramp.setPower(power);
    }



}
