package teamcode.test.Miscellanious;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import teamcode.Competition.Subsystems.ArmSystem;
import teamcode.common.AbstractOpMode;
import teamcode.common.Logger;
import teamcode.common.MecanumDriveTrain;

@Disabled
@Autonomous(name="Color")
public class ColorSensorTest extends AbstractOpMode {

    NormalizedColorSensor sensor;
    ColorRangeSensor frontRed, backRed;
    //ColorSensor color;
    DetectedElement element;
    ArmSystem arm;
    Logger logger;
    MecanumDriveTrain drive;

    @Override
    protected void onInitialize() {
        sensor = hardwareMap.get(NormalizedColorSensor.class, "WarehouseTapeSensor");
        frontRed = hardwareMap.get(ColorRangeSensor.class, "FrontColorSensorRed");
        backRed = hardwareMap.get(ColorRangeSensor.class, "BackColorSensorRed");
        //color = hardwareMap.colorSensor.get("WarehouseTapeSensor");
        element = DetectedElement.NONE;
        arm = new ArmSystem(hardwareMap, true);
        logger = new Logger(new String[]{"Red.txt", "Blue.txt", "Green.txt", "Alpha.txt"});
        drive = new MecanumDriveTrain(hardwareMap, false, null, arm);

    }

    @Override
    protected void onStart() {
        sensor.setGain(500);
        NormalizedRGBA previousColors = sensor.getNormalizedColors();
        int iterator = 0;
      //  drive.setMotorVelocity(6,6,6,6);

        while(opModeIsActive()){
            if(gamepad1.b){
                arm.preScore();
            }else if(gamepad1.a){
                arm.lowerLinkage();
            }
            if(gamepad1.right_trigger > 0.3){
                arm.intakeDumb(1.0);
            }else if(gamepad1.left_trigger > 0.3){
                arm.intakeDumb(-1.0);
            }else{
                arm.intakeDumb(0);
            }
            NormalizedRGBA colors = sensor.getNormalizedColors();
            double red = colors.red;
            double green = colors.green;
            double blue = colors.blue;
            double alpha = colors.alpha;
            if(green > 0.9){
                if(blue > 0.9){
                    element = DetectedElement.BALL;
                }else{
                    element = DetectedElement.CUBE;
                }
            }else{
                element = DetectedElement.NONE;
            }
            telemetry.addData("element", element);
            telemetry.addData("frontred", frontRed.getDistance(DistanceUnit.INCH));
            telemetry.addData("backred", backRed.getDistance(DistanceUnit.INCH));
            telemetry.addData("B", blue);
            telemetry.addData("A", alpha);
//            telemetry.addData("R", color.red());
//            telemetry.addData("G", color.green());
//            telemetry.addData("B", color.blue());
            telemetry.update();
            double dred = red - previousColors.red;
            double dblue = blue - previousColors.blue;
            double dgreen = green - previousColors.green;
            double dalpha = alpha - previousColors.alpha;
            iterator++;

//            logger.writeToLogString(0, iterator + "," + red + "\n");
//            logger.writeToLogString(1, iterator + "," + blue + "\n");
//            logger.writeToLogString(2, iterator + "," + green + "\n");
//            logger.writeToLogString(3, iterator + "," + alpha + "\n");
//            previousColors = colors;
        }
    }


    private enum DetectedElement{
        BALL, CUBE, NONE
    }
    @Override
    protected void onStop() {
      //  logger.writeLoggerToFile();
    }
}
