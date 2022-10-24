package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.concurrent.atomic.AtomicBoolean;

public class MecanumDrive {
    DcMotor fl, fr, bl, br;
    Telemetry telemetry;
    Thread driveThread;
    AtomicBoolean active;

    // creates/accesses file
    File loggingFile = AppUtil.getInstance().getSettingsFile("telemetry.txt");
    // holds data
    public String loggingString;

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        active = new AtomicBoolean();
        //TODO: Change the deviceName for each
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public int getFLPosition(){
        return fl.getCurrentPosition();
    }

    public int getFRPosition(){
        return fr.getCurrentPosition();

    }

    public int getBLPosition(){
        return bl.getCurrentPosition();

    }

    public int getBRPosition(){
        return br.getCurrentPosition();

    }


    public void setPower(Vector2D velocity, double turnValue, boolean isSwapped){
        turnValue = -turnValue;
        double direction =  velocity.getDirection();


        double power = velocity.magnitude();

        double angle = direction + 3*Math.PI / 4.0;
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        if(!isSwapped) {
            setPower((power * sin - turnValue), (power * cos + turnValue),
                    (power * cos - turnValue), (power * sin + turnValue));
        } else {
            setPower(-(power * sin - turnValue), -(power * cos + turnValue),
                    -(power * cos - turnValue), -(power * sin + turnValue));
        }
    }


    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(flPow);
        fr.setPower(frPow);
        bl.setPower(blPow);
        br.setPower(brPow);
    }
    //TODO: Change mecanudm drive so that each motor goes same amount of tics??(or ist it already set that way)
    public void goToPosition (double flPow, double frPow, double blPow, double brPow, int tics, String action) {
        //fl fr bl br

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // won't work for turns, only forward and backward

        int position = (int)(Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) +
                Math.abs(br.getCurrentPosition())) / 4;

        telemetry.addData("motorPosition", position);
        telemetry.update();

        while ((Math.abs(tics)-position) > 0) {
            setPower(flPow, frPow, blPow, brPow);
            position = (int)(Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) +
                    Math.abs(br.getCurrentPosition())) / 4;
        }

        loggingString += action.toUpperCase() + "/n";
        loggingString += "FL Position: " + getFLPosition() + "/n";
        loggingString += "FR Position: " + getFRPosition() + "/n";
        loggingString += "BL Position: " + getBLPosition() + "/n";
        loggingString += "BR Position: " + getBRPosition() + "/n";
        loggingString += "---------------------" + "/n";

        // loggingString += "Claw (Intake) Position: " + score.getClawPosition()

        setPower(0, 0, 0, 0);

    }

    public void goToPositionTest(int flTics, int frTics, int blTics, int brTics, double power, String action){
        //fl fr bl br
        active.set(true);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveThread = new Thread(){
            @Override
            public void run(){
                boolean flDone = false;
                boolean frDone = false;
                boolean blDone = false;
                boolean brDone = false;

                while(active.get()){
                    if(flDone && frDone && blDone && brDone){
                        active.set(false);
                    }else{
                        //fl
                        if((Math.abs(flTics) - Math.abs(fl.getCurrentPosition())) > 0){
                            setPower(power, power, power, power);
                        }else{
                            flDone = true;
                        }
                        //fr
                        if((Math.abs(frTics) - Math.abs(fr.getCurrentPosition())) > 0){
                            setPower(power, power, power, power);
                        }else{
                            frDone = true;
                        }
                        //bl
                        if((Math.abs(blTics) - Math.abs(bl.getCurrentPosition())) > 0){
                            setPower(power, power, power, power);
                        }else{
                            blDone = true;
                        }
                        //br
                        if((Math.abs(brTics) - Math.abs(br.getCurrentPosition())) > 0){
                            setPower(power, power, power, power);
                        }else{
                            brDone = true;
                        }
                        telemetry.addData("flPos", getFLPosition());
                        telemetry.addData("frPos", getFRPosition());
                        telemetry.addData("blPos", getBLPosition());
                        telemetry.addData("brPos", getBRPosition());
                        telemetry.update();
                    }

                }
                setPower(0,0,0,0);
            }
        };
        driveThread.start();


    }

    public void resetEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getPower(){
        return fl.getPower();
    }




    // logs string into file
    public void writeLoggerToFile(){
        try{
            PrintStream toFile = new PrintStream(loggingFile);
            toFile.println(loggingString);
        }catch(FileNotFoundException e){
            e.printStackTrace();
        }
    }


}
