package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class AprilTagTest extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    public void runOpMode() {
        initAprilTags();

        waitForStart();

        while(opModeIsActive()) {

            List<AprilTagDetection> detections = aprilTag.getDetections();

            for (AprilTagDetection tag : detections) {
                if (tag.metadata != null) {
                    telemetry.addData("Tag ID", tag.id);
                    telemetry.addData("X pos", tag.ftcPose.x);
                    telemetry.addData("Y pos", tag.ftcPose.y);
                    telemetry.addData("Z pos", tag.ftcPose.z);
                }
            }

            telemetry.update();
        }

    }

    public void initAprilTags() {
        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

    }
}
