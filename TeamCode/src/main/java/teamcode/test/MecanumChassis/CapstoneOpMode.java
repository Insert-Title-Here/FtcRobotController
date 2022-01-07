package teamcode.test.MecanumChassis;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;

@TeleOp(name = "Capstone Tele Op")
public class CapstoneOpMode extends AbstractOpMode {
    EndgameSystems endgameSystems;
    Thread capThread;
    int iterator;

    @Override
    protected void onInitialize() {
        capThread = new Thread() {
            public void run() {
                while (opModeIsActive()) {
                    capUpdate();
                }
            }
        };

        iterator = 0;
    }

    private void capUpdate() {
//        if(gamepad2.right_trigger > 0.3 || gamepad2.left_trigger > 0.3) {
//            double val = gamepad2.left_trigger - gamepad2.right_trigger;
//            endgameSystems.setCapstoneExtensionPower((Math.abs(val) >= 1) ? -1 : val);
//
//        }else{
//            endgameSystems.setCapstoneExtensionPower(0);
//        }
        telemetry.addData("lt", gamepad2.left_trigger);
        telemetry.addData("rt", gamepad2.right_trigger);
        telemetry.update();

        endgameSystems.setXCapPosition(gamepad2.left_stick_x);
        endgameSystems.setYCapPosition(gamepad2.right_stick_y);
        iterator++;
    }

    @Override
    protected void onStart() {
        capThread.start();
        while (opModeIsActive()) {

        }
    }

    @Override
    protected void onStop() {
        capThread.interrupt();
    }
}
