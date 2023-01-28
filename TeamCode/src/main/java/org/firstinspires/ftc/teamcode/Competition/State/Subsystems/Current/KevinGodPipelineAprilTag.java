package org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Competition.State.Autonomous.InUse.VisionConstants.BlueCone;
import org.firstinspires.ftc.teamcode.Competition.State.Autonomous.InUse.VisionConstants.Field1BlueLeft;
import org.firstinspires.ftc.teamcode.Competition.State.Autonomous.InUse.VisionConstants.Field1BlueRight;
import org.firstinspires.ftc.teamcode.Competition.State.Autonomous.InUse.VisionConstants.Field1RedLeft;
import org.firstinspires.ftc.teamcode.Competition.State.Autonomous.InUse.VisionConstants.Field1RedRight;
import org.firstinspires.ftc.teamcode.Competition.State.Autonomous.InUse.VisionConstants.Field2BlueLeft;
import org.firstinspires.ftc.teamcode.Competition.State.Autonomous.InUse.VisionConstants.Field2BlueRight;
import org.firstinspires.ftc.teamcode.Competition.State.Autonomous.InUse.VisionConstants.Field2RedLeft;
import org.firstinspires.ftc.teamcode.Competition.State.Autonomous.InUse.VisionConstants.Field2RedRight;
import org.firstinspires.ftc.teamcode.Competition.State.Autonomous.InUse.VisionConstants.RedCone;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class KevinGodPipelineAprilTag extends OpenCvPipeline {

    // AprilTag Setup Stuff

    int LEFT_PARK_ID = 1; // Tag ID 1 from the 36h11 family
    int MIDDLE_PARK_ID = 2;
    int RIGHT_PARK_ID = 3;

    private long nativeApriltagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    // Intrinsic Calibration Stuff
    //TODO: figure out how to actually calibrate at the correct resolution?
    double fx = 822.317; // was 578.272
    double fy = 822.317; // was 578.272
    double cx = 319.495; // was 402.145
    double cy = 242.502; // was 221.506

    // UNITS ARE METERS
    double tagsize = 0.166;
    double tagsizeX;
    double tagsizeY;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();





    public static boolean returnInput = true;

    AutoSide autoside;
    public boolean isField1 = true;


    public int contourTarget = 0;
    public boolean isNormalizing = false;
    private boolean normlizationBroke = false;

    // Define mats
    Mat ycrcb = new Mat();
    Mat temp = new Mat();


    // Define telemetry variable
    Telemetry telemetry;
    MecDriveV2 drive;

    // Define lists
    private ArrayList<Integer> xList, yList, contourLengths;

    // Define ints
    int cX, cY;
    double maxLength = 0;
    int maxLengthIndex = 0;
    int longestContourX = 0;
    int longestContourY = 0;


    // Don't really know what this thing is, but we're defining it
    Moments M;

    // Enums
    public enum ParkPos {
        LEFT,
        RIGHT,
        CENTER
    }

    public enum AutoSide {
        RED_RIGHT,
        RED_LEFT,
        BLUE_RIGHT,
        BLUE_LEFT
    }

    public enum Mode{
        SLEEVE,
        POLE,
        REDCONE,
        BLUECONE,
        RIGHTAUTOPOLE,
    }

    //public static AutoSide side = AutoSide.RED_LEFT;
    //public static boolean itsTotallyField1 = true;


    //Cone submat
    public static int topLeftXCone = 75;
    public static int topLeftYCone = 25;
    public static int bottomRightXCone = 265;
    public static int bottomRightYCone = 100;

    //RightAutoPole Submat
    public static int topLeftXRight = 250;
    public static int topLeftYRight = 50;
    public static int widthRight = 70;
    public static int heightRight = 100;



    static final Rect CONE_AREA = new Rect(
            new Point(topLeftXCone, topLeftYCone),
            new Point(bottomRightXCone, bottomRightYCone)
    );

    static final Rect RIGHTAUTOPOLE = new Rect(topLeftXRight, topLeftYRight, widthRight, heightRight);




    // Sets default values for pipelineMode and position
    //PipelineMode pipelineMode = PipelineMode.SIGNAL;
    Mode sleeveSense = Mode.SLEEVE;
    ParkPos position = ParkPos.CENTER;

    // this constructor is just so I can test at home
    public KevinGodPipelineAprilTag(Telemetry telemetry, AutoSide autoSide, boolean isField1){
        // Set up lists and telemetry
        xList = new ArrayList<>();
        yList = new ArrayList<>();
        contourLengths = new ArrayList<>();
        this.telemetry = telemetry;
        this.autoside = autoSide;

        this.isField1 = isField1;

        tagsizeX = tagsize;
        tagsizeY = tagsize;

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    public KevinGodPipelineAprilTag(Telemetry telemetry, MecDriveV2 drive, AutoSide autoSide, boolean isField1){
        // Set up lists and telemetry
        xList = new ArrayList<>();
        yList = new ArrayList<>();
        contourLengths = new ArrayList<>();
        this.telemetry = telemetry;
        this.drive = drive;
        this.autoside = autoSide;

        this.isField1 = isField1;

        tagsizeX = tagsize;
        tagsizeY = tagsize;

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    public void finalize() {
        // Might be null if createApriltagDetector() threw an exception
        if(nativeApriltagPtr != 0) {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        }
        else {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        // Check pipelineMode and run corresponding image processing
        if(sleeveSense == Mode.SLEEVE) {

            // Convert to greyscale
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

            synchronized (decimationSync) {
                if(needToSetDecimation) {
                    AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                    needToSetDecimation = false;
                }
            }

            // Run AprilTag
            detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

            synchronized (detectionsUpdateSync) {
                detectionsUpdate = detections;
            }

            boolean aprilTagFound = true;
            if (detections.size() != 0) {

                for (AprilTagDetection tag : detections) {
                    if (tag.id == LEFT_PARK_ID) {
                        position = ParkPos.LEFT;
                    } else if (tag.id == MIDDLE_PARK_ID) {
                        position = ParkPos.CENTER;
                    } else if (tag.id == RIGHT_PARK_ID) {
                        position = ParkPos.RIGHT;
                    } else {
                        aprilTagFound = false;
                    }
                }

            } else {
                aprilTagFound = false;
            }

            telemetry.addData("Park", position);

            if(aprilTagFound) {
                telemetry.addData("Status", "Good - AprilTag in frame");
            } else {
                telemetry.addData("Status", "no AprilTag found!");
            }
            telemetry.update();

        } else{

            // Convert to HSV color space
            Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2YCrCb);

            // Make binary image of yellow pixels

            if(sleeveSense == Mode.POLE || sleeveSense == Mode.RIGHTAUTOPOLE) {
                if(isField1){
                    if(autoside == AutoSide.RED_RIGHT){
                        Core.inRange(temp, new Scalar(Field1RedRight.Y1, Field1RedRight.Cr1, Field1RedRight.Cb1), new Scalar(Field1RedRight.Y2, Field1RedRight.Cr2, Field1RedRight.Cb2), temp);
                    }else if (autoside == AutoSide.RED_LEFT){
                        Core.inRange(temp, new Scalar(Field1RedLeft.Y1, Field1RedLeft.Cr1, Field1RedLeft.Cb1), new Scalar(Field1RedLeft.Y2, Field1RedLeft.Cr2, Field1RedLeft.Cb2), temp);
                    } else if (autoside == AutoSide.BLUE_RIGHT) {
                        Core.inRange(temp, new Scalar(Field1BlueRight.Y1, Field1BlueRight.Cr1, Field1BlueRight.Cb1), new Scalar(Field1BlueRight.Y2, Field1BlueRight.Cr2, Field1BlueRight.Cb2), temp);
                    } else {
                        Core.inRange(temp, new Scalar(Field1BlueLeft.Y1, Field1BlueLeft.Cr1, Field1BlueLeft.Cb1), new Scalar(Field1BlueLeft.Y2, Field1BlueLeft.Cr2, Field1BlueLeft.Cb2), temp);
                    }
                }else{
                    if(autoside == AutoSide.RED_RIGHT){
                        Core.inRange(temp, new Scalar(Field2RedRight.Y1, Field2RedRight.Cr1, Field2RedRight.Cb1), new Scalar(Field2RedRight.Y2, Field2RedRight.Cr2, Field2RedRight.Cb2), temp);
                    }else if (autoside == AutoSide.RED_LEFT){
                        Core.inRange(temp, new Scalar(Field2RedLeft.Y1, Field2RedLeft.Cr1, Field2RedLeft.Cb1), new Scalar(Field2RedLeft.Y2, Field2RedLeft.Cr2, Field2RedLeft.Cb2), temp);
                    } else if (autoside == AutoSide.BLUE_RIGHT) {
                        Core.inRange(temp, new Scalar(Field2BlueRight.Y1, Field2BlueRight.Cr1, Field2BlueRight.Cb1), new Scalar(Field2BlueRight.Y2, Field2BlueRight.Cr2, Field2BlueRight.Cb2), temp);
                    } else {
                        Core.inRange(temp, new Scalar(Field2BlueLeft.Y1, Field2BlueLeft.Cr1, Field2BlueLeft.Cb1), new Scalar(Field2BlueLeft.Y2, Field2BlueLeft.Cr2, Field2BlueLeft.Cb2), temp);
                    }
                }

            }else if(sleeveSense == Mode.BLUECONE){
                Core.inRange(temp, new Scalar(BlueCone.Y1, BlueCone.Cr1, BlueCone.Cb1), new Scalar(BlueCone.Y2, BlueCone.Cr2, BlueCone.Cb2), temp);
            }else if(sleeveSense == Mode.REDCONE){
                Core.inRange(temp, new Scalar(RedCone.Y1, RedCone.Cr1, RedCone.Cb1), new Scalar(RedCone.Y2, RedCone.Cr2, RedCone.Cb2), temp);
            }

            // Blur image to reduce noise
            Imgproc.GaussianBlur(temp, temp, new Size(5, 5), 0);

            // Find all contours in binary image
            List<MatOfPoint> contours = new ArrayList<>();

            if(sleeveSense != Mode.POLE && sleeveSense != Mode.RIGHTAUTOPOLE) {
                Imgproc.findContours(temp.submat(CONE_AREA), contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                Imgproc.rectangle(input, CONE_AREA, new Scalar(255, 92, 90), 2);
            } else if(sleeveSense == Mode.RIGHTAUTOPOLE){
                Imgproc.findContours(temp.submat(RIGHTAUTOPOLE), contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                Imgproc.rectangle(input, RIGHTAUTOPOLE, new Scalar(255, 92, 90), 2);
            } else {
                Imgproc.findContours(temp, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            }

            for(int i = 0; i < contours.size(); i++){

                // Filter out small, irrelevant contours
                if(Imgproc.contourArea(contours.get(i)) > 200) {

                    // Draw all contours to the screen
                    if(sleeveSense != Mode.POLE && sleeveSense != Mode.RIGHTAUTOPOLE) {
                        Imgproc.drawContours(input.submat(CONE_AREA), contours, i, new Scalar(230, 191, 254));
                    } else if(sleeveSense == Mode.RIGHTAUTOPOLE){
                        Imgproc.drawContours(input.submat(RIGHTAUTOPOLE), contours, i, new Scalar(230, 191, 254));
                    } else {
                        Imgproc.drawContours(input, contours, i, new Scalar(230, 191, 254));
                    }

                    // Find center of contour and add a point on the screen
                    M = Imgproc.moments(contours.get(i));
                    cX = (int)(M.m10 / M.m00);
                    cY = (int)(M.m01 / M.m00);

                    if(sleeveSense != Mode.POLE) {
                        cX += topLeftXCone;
                        cY += topLeftYCone;
                    }

                    Imgproc.circle(input, new Point(cX, cY), 3, new Scalar(0, 0, 255));

                    // Save the contour's center in a list
                    xList.add(cX);
                    yList.add(cY);

                    // Calculate the length of the contour and add it to a list
                    contourLengths.add(contours.get(i).toArray().length);

                }
            }

            // Reset maxLength so the program doesn't crash - Krish is a genious
            maxLength = 0;

            // Find largest contour
            for(int i = 0; i < xList.size() && i < contourLengths.size() && i < yList.size() && i < contours.size(); i++) {

                if(Imgproc.contourArea(contours.get(i)) > maxLength) {
                    maxLength = Imgproc.contourArea(contours.get(i));
                    maxLengthIndex = i;
                }
            }

            // Make sure the program doesn't crash if no contours are found
            if(contourLengths.size() > 0) {
                // Find x coordinate of largest contour and display it on the screen
                //TODO: Error is here
                if(maxLengthIndex < xList.size() && maxLengthIndex < yList.size()) {
                    longestContourX = xList.get(maxLengthIndex);
                    longestContourY = yList.get(maxLengthIndex);
                    Imgproc.circle(input, new Point(xList.get(maxLengthIndex), yList.get(maxLengthIndex)), 3, new Scalar(0, 255, 0));
                }
            }

            // Telemetry stuff
            /*telemetry.addData("Contour X Pos", longestContourX);
            telemetry.update();
*/
            // Clear lists
            contourLengths.clear();
            xList.clear();
            yList.clear();

            if(isNormalizing) {
                Imgproc.drawMarker(input, new Point(contourTarget, 120), new Scalar(255, 92, 90));
            } else {
                Imgproc.drawMarker(input, new Point(contourTarget, 120), new Scalar(100, 200, 200));
            }


        }
        if(returnInput) {
            return input;
        } else {
            return temp;
        }
    }

    // Get x coordinate of center of largest contour (pole)
    public int getXContour() {
        return longestContourX;
    }

    public int getYContour(){
        return longestContourY;
    }

    // Get parking position determined by signal mode
    public ParkPos getPosition() {
        return position;
    }

    public int normalize(double power, int target, int tolerance) {
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        contourTarget = target;
        isNormalizing = true;
        boolean wrongWay = false;
        int xMax = target + tolerance;
        int xMin = target - tolerance;
        double startPos = drive.avgPos();
        int startPolePosition = getXContour();



        while((getXContour() > xMax || getXContour() < xMin)) {
            if(getXContour() > xMax) {
                drive.setPowerAuto(power, MecDriveV2.MovementType.ROTATE);
            } else {
                drive.setPowerAuto(-power, MecDriveV2.MovementType.ROTATE);
            }

//            drive.setPowerAuto(power, MecDriveV2.MovementType.ROTATE);
            if(time.seconds() - startTime > 2){
                //normlizationBroke = true;
                wrongWay = true;
                break;
            }
        }
        drive.simpleBrake();

        isNormalizing = false;



        if(getXContour() < startPolePosition){
            return -(int)(startPos - drive.avgPos());

        }

        return (int)(startPos - drive.avgPos());



    }


    public int normalize(double power, int target, int tolerance, double lowerThreshold, double upperThreshold) {
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        contourTarget = target;
        isNormalizing = true;
        int xMax = target + tolerance;
        int xMin = target - tolerance;
        double startPos = drive.avgPos();
        int startPolePosition = getXContour();



        while((getXContour() > xMax || getXContour() < xMin)) {

            if(drive.getFirstAngle() < lowerThreshold){
                drive.setPowerAuto(-power, MecDriveV2.MovementType.ROTATE);
                telemetry.addData("normalize", "less than lower");


            }else if(drive.getFirstAngle() > upperThreshold){
                drive.setPowerAuto(power, MecDriveV2.MovementType.ROTATE);
                telemetry.addData("normalize", "greater than upper");


            }else{
                if(getXContour() > xMax) {
                    drive.setPowerAuto(power, MecDriveV2.MovementType.ROTATE);
                }else{
                    drive.setPowerAuto(-power, MecDriveV2.MovementType.ROTATE);
                }
                telemetry.addData("normalize", "normal");

            }


//            drive.setPowerAuto(power, MecDriveV2.MovementType.ROTATE);
            if(time.seconds() - startTime > 2){

                break;
            }



            telemetry.addData("angle", drive.getFirstAngle() * (180 / Math.PI));
            telemetry.update();
        }
        drive.simpleBrake();

        isNormalizing = false;

        if(getXContour() < startPolePosition){
            return -(int)(startPos - drive.avgPos());

        }
        return (int)(startPos - drive.avgPos());
    }


    public int normalize(double power, int target, int tolerance, double lowerThreshold, double upperThreshold, double broken) {
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        contourTarget = target;
        isNormalizing = true;
        int xMax = target + tolerance;
        int xMin = target - tolerance;
        double startPos = drive.avgPos();
        int startPolePosition = getXContour();



        while((getXContour() > xMax || getXContour() < xMin)) {

            if(drive.getFirstAngle() < lowerThreshold){
                drive.setPowerAuto(-power, MecDriveV2.MovementType.ROTATE);
                telemetry.addData("normalize", "less than lower");


            }else if(drive.getFirstAngle() > upperThreshold){
                drive.setPowerAuto(power, MecDriveV2.MovementType.ROTATE);
                telemetry.addData("normalize", "greater than upper");


            }else{
                if(getXContour() > xMax) {
                    drive.setPowerAuto(power, MecDriveV2.MovementType.ROTATE);
                }else{
                    drive.setPowerAuto(-power, MecDriveV2.MovementType.ROTATE);
                }
                telemetry.addData("normalize", "normal");

            }


//            drive.setPowerAuto(power, MecDriveV2.MovementType.ROTATE);
            if(time.seconds() - startTime > 1){
                drive.tankRotatePIDSpecial(broken, 0.3, false, 1);
                break;
            }



            telemetry.addData("angle", drive.getFirstAngle() * (180 / Math.PI));
            telemetry.update();
        }
        drive.simpleBrake();

        isNormalizing = false;

        if(getXContour() < startPolePosition){
            return -(int)(startPos - drive.avgPos());

        }
        return (int)(startPos - drive.avgPos());
    }



    public int normalize(int velocity, int target, int tolerance) {
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        contourTarget = target;
        isNormalizing = true;
        boolean wrongWay = false;
        int xMax = target + tolerance;
        int xMin = target - tolerance;
        double startPos = drive.avgPos();
        int startPolePosition = getXContour();



        while((getXContour() > xMax || getXContour() < xMin)) {
            if(getXContour() > xMax) {
                drive.setVelocity(velocity, -velocity, velocity, -velocity);
            } else {
                drive.setVelocity(-velocity, velocity, -velocity, velocity);
            }

//            drive.setPowerAuto(power, MecDriveV2.MovementType.ROTATE);
            if(time.seconds() - startTime > 2){
                //normlizationBroke = true;
                wrongWay = true;
                break;
            }
        }
        drive.simpleBrake();

        isNormalizing = false;



        if(getXContour() < startPolePosition){
            return -(int)(startPos - drive.avgPos());

        }

        return (int)(startPos - drive.avgPos());



    }


    public int normalizeStrafe(double power, int target, int tolerance) {
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        contourTarget = target;
        isNormalizing = true;
        boolean wrongWay = false;
        int xMax = target + tolerance;
        int xMin = target - tolerance;
        double startPos = drive.avgPos();
        int startPolePosition = getXContour();



        while((getXContour() > xMax || getXContour() < xMin)) {
            if(getXContour() > xMax) {
                drive.setPowerAuto(-power, MecDriveV2.MovementType.STRAFE);
            } else {
                drive.setPowerAuto(power, MecDriveV2.MovementType.STRAFE);
            }

//            drive.setPowerAuto(power, MecDriveV2.MovementType.ROTATE);
            if(time.seconds() - startTime > 2){
                //normlizationBroke = true;
                wrongWay = true;
                break;
            }
        }
        drive.simpleBrake();

        isNormalizing = false;



        if(getXContour() < startPolePosition){
            return -(int)(startPos - drive.avgPos());

        }

        return (int)(startPos - drive.avgPos());





    }


    public int normalizeStraight(double power, int target, int tolerance) {
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        contourTarget = target;
        isNormalizing = true;
        int yMax = target + tolerance;
        int yMin = target - tolerance;
        double startPos = drive.avgPos();
        int startPolePosition = getYContour();



        while((getXContour() > yMax || getXContour() < yMin)) {
            if(getXContour() > yMax) {
                drive.setPowerAuto(-power, MecDriveV2.MovementType.STRAIGHT);
            } else {
                drive.setPowerAuto(power, MecDriveV2.MovementType.STRAIGHT);
            }

//            drive.setPowerAuto(power, MecDriveV2.MovementType.ROTATE);
            if(time.seconds() - startTime > 2){
                //normlizationBroke = true;
                break;
            }
        }
        drive.simpleBrake();

        isNormalizing = false;



        if(getYContour() < startPolePosition){
            return -(int)(startPos - drive.avgPos());

        }

        return (int)(startPos - drive.avgPos());





    }




    public boolean getNormalizationBroke(){
        return normlizationBroke;
    }



    public void changeMode(Mode mode){
        sleeveSense = mode;
    }

}
