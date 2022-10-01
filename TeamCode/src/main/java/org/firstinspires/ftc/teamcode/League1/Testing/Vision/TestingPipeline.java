package org.firstinspires.ftc.teamcode.League1.Testing.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestingPipeline extends OpenCvPipeline {

    Telemetry telemetry;
    Mat temp = new Mat();
    Mat red = new Mat();
    Mat green = new Mat();
    Mat blue = new Mat();


    static final Rect MIDDLE = new Rect(
            new Point(100, 0),
            new Point(200, 100)
    );

    public TestingPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2HSV);

        Core.inRange(temp, new Scalar(50, 100, 100), new Scalar(60, 255, 255), temp);

        double mean = Core.mean(temp).val[0];

        telemetry.addData("mean ", mean);
        telemetry.update();
        //Imgproc.threshold(temp, temp, 100, 255, 0);




        /*
        Core.extractChannel(input, temp, 0);

        Core.inRange(temp, new Scalar(200), new Scalar(255), temp);


        double countRed = Core.mean(temp.submat(MIDDLE)).val[0];

        telemetry.addData("red non zero: ", countRed);

        Core.extractChannel(input, green, 1);
        Core.inRange(green, new Scalar(200), new Scalar(255), green);
        double countGreen = Core.mean(green.submat(MIDDLE)).val[0];
        telemetry.addData("green non zero: ", countGreen);

        Core.extractChannel(input, blue, 2);
        Core.inRange(blue, new Scalar(200), new Scalar(255), blue);
        double countBlue = Core.mean(blue.submat(MIDDLE)).val[0];
        telemetry.addData("blue non zero: ", countBlue);
        telemetry.update();


        if(countRed > 200){
            Imgproc.rectangle(
                    input,
                    MIDDLE,
                    new Scalar(0, 255, 0)

            );

        }else {
            Imgproc.rectangle(
                    input,
                    MIDDLE,
                    new Scalar(255, 255, 255)

            );
        }

         */



        return temp;
    }
}
