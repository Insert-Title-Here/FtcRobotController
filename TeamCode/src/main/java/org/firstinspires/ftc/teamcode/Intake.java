package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    public enum ConstantState{
        In,
        Still,
        Out,
    }



    private Servo lJoint, rJoint;
    private CRServo lIntake, rIntake;
    public ConstantState constantState;
    private ConstantState previousState;

    public Intake(HardwareMap hardwareMap){
        lJoint = hardwareMap.get(Servo.class, "lJoint");
        rJoint = hardwareMap.get(Servo.class, "rJoint");
        lIntake = hardwareMap.get(CRServo.class, "lIntake");
        rIntake = hardwareMap.get(CRServo.class, "rIntake");

        lJoint.setPosition(1);
        rJoint.setPosition(0);

        constantState = ConstantState.Still;
        previousState = ConstantState.Out;


    }



    public void clampAndRelease(boolean toClamp){
        if (toClamp) {
            lJoint.setPosition(0.9);
            rJoint.setPosition(0.1);
        } else {
            lJoint.setPosition(1);
            rJoint.setPosition(0);
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

    public void brake(){
        lIntake.setPower(0);
        rIntake.setPower(0);
    }
}
