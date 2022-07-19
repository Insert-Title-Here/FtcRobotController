package org.firstinspires.ftc.teamcode.KrishTesting.Auto;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionTesting extends OpenCvPipeline {
    Telemetry telemetry;
    Position position;
    Mat mat = new Mat();

    public enum Position{
        RIGHT,
        MIDDLE,
        LEFT,
    }

    static final Rect LEFT = new Rect(
            new Point(40, 60),
            new Point(100, 100)
    );
    static final Rect MIDDLE = new Rect(
            new Point(140, 60),
            new Point(200, 100)
    );

    static final Rect RIGHT = new Rect(
            new Point(240, 60),
            new Point(300, 100)
    );

    public VisionTesting(Telemetry t){
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT);
        Mat middle = mat.submat(MIDDLE);
        Mat right = mat.submat(RIGHT);

        Imgproc.rectangle(
                input,
                LEFT,
                new Scalar(255, 0, 0)

        );

        Imgproc.rectangle(
                input,
                MIDDLE,
                new Scalar(255, 0, 0)

        );

        Imgproc.rectangle(
                input,
                RIGHT,
                new Scalar(255, 0, 0)
        );

        double isLeft = Core.mean(left).val[0];
        double isMiddle = Core.mean(middle).val[0];
        double isRight = Core.mean(right).val[0];




        telemetry.addData("left val[0]: ", isLeft);
        telemetry.addData("middle val[0]: ", isMiddle);
        telemetry.addData("right val[0]: ", isRight);

        if(isLeft > 60){
            position = Position.LEFT;
        }else if(isMiddle > 60){
            position = Position.MIDDLE;
        }else{
            position = Position.RIGHT;
        }

        telemetry.addData("position: ", getPosition());
        telemetry.update();


        return input;

    }

    public Position getPosition(){
        return position;
    }
}
