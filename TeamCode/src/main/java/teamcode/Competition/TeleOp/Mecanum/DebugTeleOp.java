package teamcode.Competition.TeleOp.Mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Debug;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Vector2D;

@Disabled
@TeleOp(name="debug")
public class DebugTeleOp extends AbstractOpMode {
    MecanumDriveTrain drive;
    ArmSystem arm;
    Thread armThread;
    private long scoredSampleTime;


    @Override
    protected void onInitialize() {
        drive = new MecanumDriveTrain(hardwareMap);
        arm = new ArmSystem(hardwareMap, true);
        armThread = new Thread(){
            public void run(){
                while(opModeIsActive()){
                    armUpdate();
                }
            }
        };
        startTime = 0;
        scoredSampleTime = 0;

    }
    double startTime;
    private void armUpdate() {
        if(gamepad1.right_trigger > 0.3) {
            startTime = AbstractOpMode.currentOpMode().time;
            while (gamepad1.right_trigger > 0.3) {
                double elapsedTime = time - startTime;
                if (elapsedTime < 0.5) {
                    arm.lowerLinkage();
                } else {
                    arm.intakeDumb(1.0);
                }

            }
        }else if(gamepad1.x){
            arm.score();
        } else if(gamepad1.b) {
            arm.preScore();
        }else if(gamepad1.dpad_down){
            arm.retract();
        } else if(gamepad1.left_trigger > 0.3) {
                arm.raise(Constants.TOP_POSITION);

        }else if(gamepad1.a){
            while(gamepad1.a){
                arm.intakeDumb(-1.0);
            }
        }else{
            arm.intakeDumb(0);
        }

    }

    @Override
    protected void onStart() {
        armThread.start();
        while(opModeIsActive()){
            drive.setPower(new Vector2D(-gamepad1.left_stick_y, gamepad1.left_stick_x),  0.7 * gamepad1.right_stick_x);
        }


    }

    @Override
    protected void onStop() {

    }
}
