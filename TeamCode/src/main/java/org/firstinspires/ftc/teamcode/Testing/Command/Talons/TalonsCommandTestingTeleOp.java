package org.firstinspires.ftc.teamcode.Testing.Command.Talons;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Testing.Command.Talons.Command.CarouselCommand;
import org.firstinspires.ftc.teamcode.Testing.Command.Talons.CommandGroup.TalonsLiftingSequence;
import org.firstinspires.ftc.teamcode.Testing.Command.Talons.CommandGroup.TalonsRampScore;
import org.firstinspires.ftc.teamcode.Testing.Command.Talons.CommandGroup.TalonsSlideScore;
import org.firstinspires.ftc.teamcode.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Vector2D;

import java.io.FileNotFoundException;

@TeleOp
public class TalonsCommandTestingTeleOp extends CommandOpMode {

    private RobotT robot;
    public MecanumDriveTrain drive;
    public TalonsIntake intake;
    public TalonsScoringSystem lift;
    public TalonsEndGame endgameSystem;

    Thread driveThread;

    private final double NORMAL_LINEAR_MODIFIER = 0.45;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;


    ElapsedTime timer;
    private boolean timerFlag;
    private volatile boolean  endgame;
    private volatile boolean swap;

    public enum OpModeType {
        TELEOP, AUTO
    }



    @Override
    public void initialize() {

        driveThread = new Thread(){
            @Override
            public void run(){
                waitForStart();
                while(opModeIsActive()) {
                    if(gamepad1.dpad_down && swap){
                        endgame = !endgame;
                        CommandScheduler.getInstance().reset();
                        endgameSystem.brakeCarousel();
                        swap = false;
                    }else{
                        swap = true;
                    }

                    if(!endgame) {
                        if (gamepad1.right_bumper) { // replace this with a button for sprint
                            drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
                        } else {
                            drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
                        }
                    }else{

                        double yPos = endgameSystem.getYPosition();

                        endgameSystem.setxCapPower(-gamepad1.right_stick_x / 3);
                        /*
                        if(gamepad1.right_stick_y > 0.1){
                            endgameSystem.jankUpY();
                        }else if(gamepad1.right_stick_y < -0.1){
                            endgameSystem.jankDownY();
                        }

                         */

                        endgameSystem.setYPosition(yPos - endgameSystem.map(gamepad1.right_stick_y, -1, 1, -0.0010, 0.0010));

                        if(gamepad1.right_trigger > 0.1){
                            endgameSystem.setCappingPower(-gamepad1.right_trigger);
                        }else{
                            endgameSystem.setCappingPower(0);
                        }

                        if(gamepad1.left_trigger > 0.1){
                            endgameSystem.setCappingPower(gamepad1.left_trigger);
                        }else{
                            endgameSystem.setCappingPower(0);
                        }


                    }
                }

            }
        };



        try {
            robot = new RobotT(hardwareMap);
            drive = new MecanumDriveTrain(hardwareMap);

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }


        intake = new TalonsIntake(hardwareMap);
        lift = new TalonsScoringSystem(hardwareMap);
        endgameSystem = new TalonsEndGame(hardwareMap);
        timer = new ElapsedTime();
        timerFlag = true;
        endgame = false;
        swap = true;

        lift.linkageSetPosition(0.35);
        driveThread.start();

    }



    @Override
    public void run() {
        super.run();
        robot.update();



        if(!endgame) {
            if (gamepad1.a) {

                //robot.intake.runIntake = !robot.intake.runIntake;
                //.lift.up();
                //robot.lift.extend(0.5);
                //switchIntake = false;
                schedule(new TalonsLiftingSequence(OpModeType.TELEOP, intake, lift, gamepad1, robot));
            }

            if (gamepad1.b) {
                schedule(new TalonsRampScore(OpModeType.TELEOP, lift));

            }

            if (gamepad1.y) {
                schedule(new TalonsSlideScore(OpModeType.TELEOP, lift, gamepad1));

                //lift.extend(0.5, gamepad1);

                //robot.runToPosition(5000, 0.5);
            }


            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-gamepad1.left_trigger);
            } else {
                intake.setPower(0);
            }
        }else{
            schedule(new SequentialCommandGroup(new CarouselCommand(endgameSystem, gamepad1)));
        }






    /*
        if(robot.rgba != null) {
            telemetry.addData("Normalized rgba alpha", robot.rgba.alpha);
            telemetry.addData("Normalized rgba red", robot.rgba.red);
            telemetry.addData("Normalized rgba green", robot.rgba.green);
            telemetry.addData("Normalized rgba blue", robot.rgba.blue);
        }
        telemetry.addData("Distance", robot.color.getDistance(DistanceUnit.INCH));
        telemetry.addData("extended: ", robot.lift.isExtended());
        telemetry.addData("lift encoder: ", robot.getSpecificEncoderValue(2,false));
        telemetry.addData("lift : ", robot.lift.getPosition(false));
        telemetry.addData("run to position (extend): ", Math.abs(Math.abs(robot.lift.liftEncoder.getCurrentPosition()) - 5000));

     */
        telemetry.addData("Endgame: ", endgame);
        telemetry.update();







    }







}
