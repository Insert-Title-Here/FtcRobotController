package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TeleOp.TestTeleOp;

public class MaintainLiftPosition implements Runnable{
    @Override
    public void run() {
        int targPos = TestTeleOp.getTargetPosition();
        int targPosR = TestTeleOp.getRightTargetPosition();
        DcMotor lift = TestTeleOp.getLift();
        DcMotor rightLift = TestTeleOp.getRightLift();
        while(TestTeleOp.liftIsStill()){
            int currPos = lift.getCurrentPosition();
            int currPosR = rightLift.getCurrentPosition();
            lift.setTargetPosition(targPos);
            rightLift.setTargetPosition(targPosR);
        }
    }
}
