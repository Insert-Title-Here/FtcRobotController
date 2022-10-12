package org.firstinspires.ftc.teamcode.League1.Testing.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SignalPipeline extends OpenCvPipeline {

    public enum ParkPos {
        LEFT,
        RIGHT,
        CENTER
    }

    Telemetry telemetry;
    Mat temp = new Mat();
    Mat ycrcb = new Mat();



    static final Rect MIDDLE = new Rect(
            new Point(125, 35),
            new Point(160, 95)
    );

    public SignalPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    volatile ParkPos position = ParkPos.CENTER;

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(ycrcb, temp, 0);
        Core.inRange(temp, new Scalar(160), new Scalar(190), temp);


        double countY = Core.mean(temp.submat(MIDDLE)).val[0];

        telemetry.addData("countY", countY);

        Core.extractChannel(ycrcb, temp, 1);
        Core.inRange(temp, new Scalar(130), new Scalar(200), temp);
        double countCr = Core.mean(temp.submat(MIDDLE)).val[0];
        telemetry.addData("countCr", countCr);

        Core.extractChannel(ycrcb, temp, 2);
        Core.inRange(temp, new Scalar(130), new Scalar(170), temp);
        double countCb = Core.mean(temp.submat(MIDDLE)).val[0];
        telemetry.addData("countCb", countCb);

        if(countY > 100 && countCb < 100) {
            telemetry.addData("Color", "Yellow");
            position = ParkPos.LEFT;
        } else if(countCr > 200 && countCb > 200) {
            telemetry.addData("Color", "Magenta");
            position = ParkPos.RIGHT;
        } else {
            telemetry.addData("Color", "Cyan");
            position = ParkPos.CENTER;
        }

        telemetry.update();


        if(position == ParkPos.LEFT){
            Imgproc.rectangle(
                    input,
                    MIDDLE,
                    new Scalar(255, 255, 0),
                    3

            );

        }else if(position == ParkPos.RIGHT){
            Imgproc.rectangle(
                    input,
                    MIDDLE,
                    new Scalar(255, 0, 255),
                    3

            );

        }else{
            Imgproc.rectangle(
                    input,
                    MIDDLE,
                    new Scalar(0, 155, 255),
                    3

            );
        }

        return input;

    }

    public ParkPos getPosition() {
        return position;
    }
}
