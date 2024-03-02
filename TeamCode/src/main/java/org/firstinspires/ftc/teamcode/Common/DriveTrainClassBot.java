package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrainClassBot {
    DcMotorEx left, right;

    public DriveTrainClassBot(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, "motor 1");
        right = hardwareMap.get(DcMotorEx.class, "motor 2");


        //robot.setShouldUpdate(false);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


    }
    public void setPower(double leftPower, double rightPower){
        left.setPower(leftPower);
        right.setPower(rightPower);
    }
}
