package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    public enum ConstantState{
        In,
        Still,
        Out,
    }



    private Servo lJoint, rJoint, linkage;
    private CRServo lIntake, rIntake;
    public ConstantState constantState;
    private ConstantState previousState;
    //  private ColorRangeSensor color;

    public Intake(HardwareMap hardwareMap){
        lJoint = hardwareMap.get(Servo.class, "LeftClamp");
        rJoint = hardwareMap.get(Servo.class, "RightClamp");
        lIntake = hardwareMap.get(CRServo.class, "LeftIntake");
        rIntake = hardwareMap.get(CRServo.class, "RightIntake");
        linkage = hardwareMap.get(Servo.class, "Linkage");

        linkage.setPosition(0);


        lJoint.setPosition(0.87); //was 0
        rJoint.setPosition(0.43); //was 1

        constantState = ConstantState.Still;
        previousState = ConstantState.Out;


    }



    public void clampAndRelease(boolean toClamp){
        if (toClamp) {
            //lJoint.setPosition(0.82); //was 0.9
            rJoint.setPosition(0.6); //was 0.1
        } else {
            //lJoint.setPosition(0.87); //was 0
            rJoint.setPosition(0.43); //was 0.5
        }
    }

    public void setPower(boolean rT, double power){
        if (rT) {
            lIntake.setPower(power);
            rIntake.setPower(-power);
        } else{
            lIntake.setPower(-power);
            rIntake.setPower(power);
        }
    }

    public void setPower(double power) {
        lIntake.setPower(power);
        rIntake.setPower(-power);
    }

    public void setLinkage(double position) {
        linkage.setPosition(position);
    }

    public void shiftConstantState(){
        if(constantState == ConstantState.In){
            constantState = ConstantState.Still;
            previousState = ConstantState.In;
        }else if(constantState == ConstantState.Out){
            constantState = ConstantState.Still;
            previousState = ConstantState.Out;
        }else{
            if(previousState == ConstantState.Out){
                constantState = ConstantState.In;

            }else if(previousState == ConstantState.In){
                constantState = ConstantState.Out;
            }

            previousState = ConstantState.Still;
        }
    }

    public void brake() {
        lIntake.setPower(0);
        rIntake.setPower(0);
    }

}
