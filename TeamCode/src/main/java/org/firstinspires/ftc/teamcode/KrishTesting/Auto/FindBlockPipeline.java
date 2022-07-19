package org.firstinspires.ftc.teamcode.KrishTesting.Auto;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class FindBlockPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    boolean inMiddle;




    static final Rect MIDDLE = new Rect(
            new Point(140, 30),
            new Point(200, 176)
    );



    public FindBlockPipeline(Telemetry t){
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat middle = mat.submat(MIDDLE);



        Imgproc.rectangle(
                input,
                MIDDLE,
                new Scalar(255, 0, 0)

        );



        double isMiddle = Core.mean(middle).val[0];

        if(isMiddle > 35){
            inMiddle = true;
        }else{
            inMiddle = false;
        }




        telemetry.addData("middle val[0]: ", isMiddle);

        telemetry.update();


        return input;

    }

    public boolean getInMiddle(){
        return inMiddle;
    }
}
