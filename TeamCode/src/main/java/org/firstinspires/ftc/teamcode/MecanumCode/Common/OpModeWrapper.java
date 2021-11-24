package org.firstinspires.ftc.teamcode.MecanumCode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class OpModeWrapper extends LinearOpMode {

    private static OpModeWrapper opMode;

    public static OpModeWrapper currentOpMode() {
        return opMode;
    }

    @Override
    public final void runOpMode() {
        opMode = this;
        try {
            onInitialize();
            waitForStart();
            onStart();
            onStop();
        } catch (Exception e) {
            telemetry.addData("", e);
            telemetry.update();
            // give time to read exception
            try {
                Thread.sleep(5000);
            } catch (InterruptedException interruptedException) {
                interruptedException.printStackTrace();
            }
        }
    }

    protected abstract void onInitialize();

    protected abstract void onStart();

    protected abstract void onStop();

}
