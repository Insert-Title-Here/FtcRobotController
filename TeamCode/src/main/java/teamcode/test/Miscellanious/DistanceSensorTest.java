package teamcode.test.Miscellanious;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.Competition.Subsystems.EndgameSystems;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.common.Vector2D;

@Autonomous(name="DistanceSensor")
public class DistanceSensorTest extends AbstractOpMode {
    MecanumDriveTrain drive;
    EndgameSystems system;
    private ArmSystem arm;
    NormalizedColorSensor frontSensor, backSensor;
    private int FRONT_GAIN  = 100, BACK_GAIN = 420;
    ColorSensor front, back;

    private boolean previousDpadDown1, previousDpadUp1, previousDpadDown2, previousDpadUp2;

    @Override
    protected void onInitialize() {
        arm = new ArmSystem(hardwareMap, false);
        drive = new MecanumDriveTrain(hardwareMap, false, system, arm);
        system = new EndgameSystems(hardwareMap, true);
          //frontSensor = hardwareMap.get(NormalizedColorSensor.class, "FrontColorSensor");
//        backSensor = hardwareMap.get(NormalizedColorSensor.class, "BackColorSensor");
//        frontSensor.setGain(FRONT_GAIN);
//        backSensor.setGain(BACK_GAIN);
//        previousDpadDown1 = gamepad1.dpad_down;
//        previousDpadUp1 = gamepad1.dpad_up;
//        previousDpadDown2 = gamepad2.dpad_down;
//        previousDpadUp2 = gamepad2.dpad_up;

    }

    @Override
    protected void onStart() {

        system.scoreDuck();
        //drive.arcDriving(Vector2D.fromAngleMagnitude(Math.toRadians(45), 6), 6, 500, 45);
        while(opModeIsActive()){
//            NormalizedRGBA frontRGBA = frontSensor.getNormalizedColors();
//            NormalizedRGBA backRGBA = backSensor.getNormalizedColors();
//            telemetry.addData("FRed", frontRGBA.red);
//            telemetry.addData("FGreen", frontRGBA.green);
//            telemetry.addData("FBlue", frontRGBA.blue);
//            telemetry.addData("", "--------------------");
//            telemetry.addData("BRed", backRGBA.red);
//            telemetry.addData("BGreen", backRGBA.green);
//            telemetry.addData("BBlue", backRGBA.blue);
//            telemetry.addData("", "--------------------");
//            telemetry.addData("FGain", frontSensor.getGain());
//            telemetry.addData("BGain", backSensor.getGain());
//            telemetry.update();
//
//
//
//            previousDpadDown1 = gamepad1.dpad_down;
//            previousDpadUp1 = gamepad1.dpad_up;
//            previousDpadDown2 = gamepad2.dpad_down;
//            previousDpadUp2 = gamepad2.dpad_up;


        }


//        for(int i = 0; i < 4; i++) {
//            drive.driveColorSensor(0.3);
//            Utils.sleep(500);
//            arm.raise(Constants.TOP_POSITION);
//            arm.score();
//            Utils.sleep(500);
//            arm.retract();
//            Utils.sleep(500);
//        }

        //drive.strafeDistanceSensor(0.5, 0);
        //drive.moveDistanceDEVelocity(500, 0,12);
        //drive.strafeDistanceSensor(1.0, Math.toRadians(90));
    }

    @Override
    protected void onStop() {

    }


}
