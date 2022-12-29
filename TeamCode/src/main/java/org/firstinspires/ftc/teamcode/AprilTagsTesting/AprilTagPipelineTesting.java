

package org.firstinspires.ftc.teamcode.AprilTagsTesting;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

class AprilTagPipelineTesting extends OpenCvPipeline {
    public enum parkPos {
        LEFT,
        CENTER,
        RIGHT
    }

    public parkPos position = parkPos.LEFT;

    Telemetry telemetry;

    int LEFT_PARK_ID = 1; // Tag ID 18 from the 36h11 family
    int MIDDLE_PARK_ID = 2;
    int RIGHT_PARK_ID = 3;

    private long nativeApriltagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    Scalar blue = new Scalar(7,197,235,255);
    Scalar red = new Scalar(255,0,0,255);
    Scalar green = new Scalar(0,255,0,255);
    Scalar white = new Scalar(255,255,255,255);

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

    public AprilTagPipelineTesting(Telemetry telemetry) {
        tagsizeX = tagsize;
        tagsizeY = tagsize;
        this.telemetry = telemetry;

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

        return input;
    }

    public ArrayList<AprilTagDetection> getLatestDetections()
    {
        return detections;
    }

    public parkPos getParkPos() {
        boolean parkPosFound = true;
        if (detections.size() != 0) {

            for (AprilTagDetection tag : detections) {
                if (tag.id == LEFT_PARK_ID) {
                    position = AprilTagPipelineTesting.parkPos.LEFT;
                } else if (tag.id == MIDDLE_PARK_ID) {
                    position = AprilTagPipelineTesting.parkPos.CENTER;
                } else if (tag.id == RIGHT_PARK_ID) {
                    position = AprilTagPipelineTesting.parkPos.RIGHT;
                } else {
                    parkPosFound = false;
                }
            }

        } else {
            parkPosFound = false;
        }

        telemetry.addData("Park", position);

        if(parkPosFound) {
            telemetry.addData("Status", "Good - AprilTag in frame");
        } else {
            telemetry.addData("Status", "no AprilTag found!");
        }
        telemetry.update();

        return position;
    }

}