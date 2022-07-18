package org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TalonsIntake {
    private DcMotor intake;
    boolean runIntake;

    public TalonsIntake (HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class, "intake");
        runIntake = false;
    }

    public void setPower(double power){
        intake.setPower(power);
    }

    public void intake(){
        setPower(0.5);
    }

    public void outtake(){
        setPower(-0.5);
    }

    public void brake(){
        setPower(0);
    }


}
