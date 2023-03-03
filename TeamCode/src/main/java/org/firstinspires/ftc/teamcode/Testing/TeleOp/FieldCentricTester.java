package org.firstinspires.ftc.teamcode.Testing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Vector2D;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Used.MecDrive;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.MecDriveV2;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.ScoringSystemV2EpicLift;

@Disabled
@TeleOp(name = "New Drive Test")
public class FieldCentricTester extends LinearOpMode {

    MecDriveV2 drive;




    //PassivePower passive;




    @Override
    public void runOpMode() throws InterruptedException {
        //Initializing flags



        //Feed forward is going to be off
        //passive = PassivePower.ZERO;

        //robot = new Robot(hardwareMap);

        drive = new MecDriveV2(hardwareMap, false, telemetry, true);
        //systems = new EndgameSystems(hardwareMap);





        waitForStart();

        //score.setLinkagePosition(Constants.linkageUpV2);




        while (opModeIsActive()) {

            //N S E W Drive
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = gamepad1.left_stick_y;

            if (Math.abs(leftStickX) > Math.abs(leftStickY)) {
                leftStickY = 0;

            } else if (Math.abs(leftStickY) > Math.abs(leftStickX)) {
                leftStickX = 0;

            } else {
                leftStickY = 0;
                leftStickX = 0;
            }

            /*if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(leftStickX * Constants.SPRINT_LINEAR_MODIFIER, leftStickY * Constants.SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.SPRINT_ROTATIONAL_MODIFIER, false);
            } else*/

            drive.setPower(new Vector2D(leftStickX/* * Constants.NORMAL_LINEAR_MODIFIER*/, leftStickY/* * Constants.NORMAL_LINEAR_MODIFIER*/), gamepad1.right_stick_x * Constants.NORMAL_ROTATIONAL_MODIFIER, false);


            //Telemetry

            /*telemetry.addData("lMotor", -1 * score.getLeftEncoderPos());
            telemetry.addData("rMotor", score.getRightEncoderPos());
            telemetry.addData("distance: ", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("distanceRed", distance.getNormalizedColors().red);
            telemetry.addData("distanceBlue", distance.getNormalizedColors().blue);
            telemetry.addData("autoLinkageFlag", autoLinkageFlag);
            telemetry.addData("grabbingFlag", grabFlag);
            telemetry.addData("manualFlag", manualFlag);
            telemetry.addData("shiftLinkageFlag", shiftLinkageFlag);
            telemetry.addData("extended", score.isExtended());
            //telemetry.addData("colorRed: ", color.getNormalizedColors().red);
            //telemetry.addData("colorBlue: ", color.getNormalizedColors().blue);
            telemetry.addData("rightServoTarget", score.getRightLinkage());
            telemetry.addData("leftServoTarget", score.getLeftLinkage());
            //telemetry.addData("passive", passive);
            telemetry.addData("coneStack", score.getConeStack());
            telemetry.addData("rip robot", liftBrokenMode);
            telemetry.update();
*/

        }


        //Stop
        drive.simpleBrake();

        //score.setLinkagePositionLogistic(0.25, 500);
        //score.setLinkagePositionLogistic(Constants.linkageDownV2, 300, 100);
        //score.setLinkagePositionLogistic(0.8, 500);


        //score.setGrabberPosition(Constants.open - 0.15);
    }
}
