package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor {
    ColorRangeSensor color;
    Telemetry telemetry;
    MecanumDrive drive;
    ScoringSystem score;
    double radians;

    public ColorSensor(HardwareMap hardwareMap, Telemetry telemetry) {
        color = hardwareMap.get(ColorRangeSensor.class, "color");
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap);
        this.telemetry = telemetry;

    }


    public int currentBlueColor() {
        return color.blue(); // if current color is really high // 410
    }

    public int currentRedColor() {
        return color.red(); // if current color is really high // 177
    }

    public void findTapeGrabCone() {
        while(currentBlueColor() < 70){ //blue tape TODO: get a num for "70"
            drive.goToPosition(0, 0.8, 0, 0.8);
            if (drive.avgPosition() > 700) {
                drive.goToPosition(0.8, 0, 0.8, 0);
            }
        }

        drive.goToPosition(0.4, 0.4, 0.4, 0.4);
        if (color.getDistance(DistanceUnit.CM) < 3) { //blue tape
            // stop driving
            drive.setPower(0, 0, 0, 0);
            // grab cone
            score.setClawPosition(0.24);
            // lift up
            score.goToPosition(600, 0.6);
            // backup
            drive.goToPosition(-0.4, -0.4, -0.4, -0.4, 200, "backwards");
            //turn
            radians = 3.14 * 7 / 6;
            drive.turn(radians, 0.5); // TODO: make sure drive.turn works
            telemetry.addData("red", currentConeRedColor());
            telemetry.addData("blue", currentConeBlueColor());
            telemetry.update();
        }

    }



    public void findCone() {

    }
}
