package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TeleOp.TestTeleOp;

public class MaintainLiftPosition implements Runnable{
    @Override
    public void run() {
        DcMotor lift = TestTeleOp.getLift();
        while(TestTeleOp.liftIsStill()){
            lift.setTargetPosition(TestTeleOp.getTargetPosition());
        }
    }
}
