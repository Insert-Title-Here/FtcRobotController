package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    /*public enum ServoPosition{
        Clamped,
        Open
    }

     */

    Servo lJoint, rJoint;
    CRServo lIntake, rIntake;

    public Intake(HardwareMap hardwareMap){
        lJoint = hardwareMap.get(Servo.class, "lJoint");
        rJoint = hardwareMap.get(Servo.class, "rJoint");
        lIntake = hardwareMap.get(CRServo.class, "lIntake");
        rIntake = hardwareMap.get(CRServo.class, "rIntake");

        lJoint.setPosition(1);
        rJoint.setPosition(0);


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

    public void brake(){
        lIntake.setPower(0);
        rIntake.setPower(0);
    }
}
