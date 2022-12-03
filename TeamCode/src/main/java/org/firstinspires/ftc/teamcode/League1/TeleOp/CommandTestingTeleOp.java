package org.firstinspires.ftc.teamcode.League1.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Common.Vector2D;
import org.firstinspires.ftc.teamcode.League1.Subsystems.EndgameSystems;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.FeedForwardCommand;
import org.firstinspires.ftc.teamcode.League1.TeleOp.CommandGroup.GrabAndScore;
import org.firstinspires.ftc.teamcode.League1.TeleOp.CommandGroup.ResetScoringGroup;

import java.util.concurrent.atomic.AtomicBoolean;

//TODO: FIX THIS

@Disabled
@TeleOp
public class CommandTestingTeleOp extends CommandOpMode {

    private GamepadEx driver, beaconMechDriver;
    //AtomicBoolean initializeRobot;
    //Robot robot;


    private MecDrive drive;
    private ScoringSystemCommand lift;
    private EndgameSystems endgameSystem;
    private Constants constants;
    private ColorRangeSensor distance, color;





    @Override
    public void initialize() {
        /*initializeRobot = new AtomicBoolean(true);
        robot = new Robot(hardwareMap);*/

        //Gamepad extensions
        driver = new GamepadEx(gamepad1);
        beaconMechDriver = new GamepadEx(gamepad2);

        //Initializing objects
        constants = new Constants();
        drive = new MecDrive(hardwareMap, false, telemetry);
        lift = new ScoringSystemCommand(hardwareMap, constants);
        endgameSystem = new EndgameSystems(hardwareMap);

        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        color = hardwareMap.get(ColorRangeSensor.class, "color");

        //TODO: might need to decrease gain by some
        color.setGain(300);
        distance.setGain(300);


        //TODO: Check all the commands and command groups
        //Close Grabber and Lift up linkage
        //driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(new GrabAndLinkageUpGroup(lift, constants, hardwareMap, robot));

        //Lift up slides to medium height
        //driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new LiftCommand(lift, constants, hardwareMap, robot, ScoringSystem.ExtensionHeight.MEDIUM, 0.5));

        //Lift up slides to height height
        //driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new LiftCommand(lift, constants, hardwareMap, robot, ScoringSystem.ExtensionHeight.HIGH, 0.5));

        //Slides back to zero, open grabber and linkage goes down
        //driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new ResetScoringGroup(lift, constants, hardwareMap, robot));

        //Testing value for up linkage
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ResetScoringGroup(lift, constants));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new GrabAndScore(lift, constants));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> lift.changeMode(ScoringSystemCommand.ScoringMode.LOW)));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> lift.changeMode(ScoringSystemCommand.ScoringMode.MEDIUM)));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> lift.changeMode(ScoringSystemCommand.ScoringMode.HIGH)));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> lift.changeMode(ScoringSystemCommand.ScoringMode.ULTRA)));


        lift.setLinkagePosition(constants.linkageDown);
        lift.setGrabberPosition(constants.open);


    }



    @Override
    public void run() {
        ParallelCommandGroup group = new ParallelCommandGroup();
        /*if(initializeRobot.get()){
            robot.start;
            initializeRobot.set(false);
        }*/



        super.run();


        //NSEW DRIVE
        double leftStickX = driver.getLeftX();
        double leftStickY = -1 * driver.getLeftY();

        if(Math.abs(leftStickX) > Math.abs(leftStickY)){
            leftStickY = 0;

        }else if(Math.abs(leftStickY) > Math.abs(leftStickX)){
            leftStickX = 0;

        }else{
            leftStickY = 0;
            leftStickX = 0;
        }

        //Could try out a ramp up sprint
        if (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            drive.setPower(new Vector2D(leftStickX * constants.SPRINT_LINEAR_MODIFIER, leftStickY * constants.SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * constants.SPRINT_ROTATIONAL_MODIFIER, false);
        } else {
            drive.setPower(new Vector2D(leftStickX * constants.NORMAL_LINEAR_MODIFIER, leftStickY * constants.NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * constants.NORMAL_ROTATIONAL_MODIFIER, false);
        }


        /*
        //TODO:Tune this value later
        if(robot.getRGBA(true).red > 500){
            schedule(new GrabAndLinkageUpGroup(lift, constants, hardwareMap, robot));

        }
        */


        //Trying out taking out distance
        if(distance.getDistance(DistanceUnit.CM) < 6.5/*distance.getNormalizedColors().red > 0.7 && distance.getNormalizedColors().blue > 0.7
                */&& !lift.isGrabbing()){
            group.addCommands(new GrabAndScore(lift, constants));
            lift.setGrabbing(true);

        }

        if(lift.isExtended()){
            group.addCommands(new FeedForwardCommand(lift, constants, hardwareMap, driver));
        }


        schedule(group);


        telemetry.update();
    }
}