package org.firstinspires.ftc.teamcode.Testing.Vision.Pipelines;

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
    Mat ycrcb = new Mat();
    Mat purple = new Mat();
    Mat green = new Mat();
    Mat brown = new Mat();
    Mat black = new Mat();


    static final Rect MIDDLE = new Rect(
            new Point(125, 35),
            new Point(160, 95)
    );

    public TestingPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {

        //Core.extractChannel(input, green, 1);
        //Core.inRange(green, new Scalar(254), new Scalar(255), green);
        //Core.inRange(input, new Scalar(255, 255, 255), new Scalar(0, 0, 0), green);

        //Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2HSV);
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
        } else if(countCr > 200 && countCb > 200) {
            telemetry.addData("Color", "Magenta");
        } else {
            telemetry.addData("Color", "Cyan");
        }

        //Imgproc.cvtColor(input, black, Imgproc.COLOR_RGB2GRAY);
        //Yellow
        //Core.inRange(temp, new Scalar(25, 120, 120), new Scalar(40, 255, 255), temp);

        //Green
        //Core.inRange(temp, new Scalar(70, 60, 60), new Scalar(100, 255, 255), green);

        //Purple
        //Core.inRange(temp, new Scalar(120, 60, 60), new Scalar(130, 255, 255), purple);

        //Brown
        //Core.inRange(temp, new Scalar(30, 0, 0), new Scalar(40, 80, 50), brown);


        //Black
        //Core.inRange(black, new Scalar(0), new Scalar(25), black);
        /*


        List<MatOfPoint> contours = new ArrayList<>();

        MatOfPoint contour1 = contours.get(0);
        MatOfPoint contour2;

        //Find out last two things
        Imgproc.findContours(temp, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);


        for(int i = 0; i < contours.size(); i++)
            telemetry.addData("contour" + i + ": ", contours.get(i).toArray().length);
        }


         */


        /*
        double countGreen = Core.mean(green.submat(MIDDLE)).val[0];
        double countPurple = Core.mean(purple.submat(MIDDLE)).val[0];
        double countBrown = Core.mean(brown.submat(MIDDLE)).val[0];
        double countBlack = Core.mean(black.submat(MIDDLE)).val[0];


        telemetry.addData("green", countGreen);
        telemetry.addData("purple", countPurple);
        telemetry.addData("brown", countBrown);
        telemetry.addData("black", countBlack);

         */



        telemetry.update();


        if(countY > 20){
            Imgproc.rectangle(
                    input,
                    MIDDLE,
                    new Scalar(0, 255, 0),
                    3

            );

        }else{
            Imgproc.rectangle(
                    input,
                    MIDDLE,
                    new Scalar(0, 0, 0),
                    3

            );

        }


/*
        if(countGreen > 50){
            Imgproc.rectangle(
                    input,
                    MIDDLE,
                    new Scalar(0, 255, 0),
                    3

            );

        }else if(countPurple > 50){
            Imgproc.rectangle(
                    input,
                    MIDDLE,
                    new Scalar(255, 0, 255),
                    3

            );

        }else if(countBrown > 17){
            Imgproc.rectangle(
                    input,
                    MIDDLE,
                    new Scalar(130, 70, 0),
                    3

            );

        }else if(countBlack > 150){
            Imgproc.rectangle(
                    input,
                    MIDDLE,
                    new Scalar(0, 0, 0),
                    3

            );

        }else {
            Imgproc.rectangle(
                    input,
                    MIDDLE,
                    new Scalar(255, 255, 255),
                    3

            );
        }

 */




        return input;




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



    }
}
