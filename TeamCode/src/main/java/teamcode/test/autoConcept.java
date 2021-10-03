package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import teamcode.common.AbstractOpMode;
import teamcode.common.Utils;

@Autonomous(name="startShoot")
public class autoConcept extends AbstractOpMode {

    @Override
    protected void onInitialize() {
        }

    @Override
    protected void onStart() {
        while(opModeIsActive()) {
            if(gamepad1.x){
                Utils.sleep(250);
            }
            telemetry.update();
        }

        //shooter.shoot(3, 30000);

    }

    @Override
    protected void onStop() {

    }
}
