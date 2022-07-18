package org.firstinspires.ftc.teamcode.KrishTesting.Command.NewBot;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.KrishTesting.RobotK;
import org.firstinspires.ftc.teamcode.Vector2D;

import java.io.FileNotFoundException;

public class CommandTestingTeleOp extends CommandOpMode {

    private RobotK robot;
    private final double NORMAL_LINEAR_MODIFIER = 0.45;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 0.75;



    @Override
    public void initialize() {
        try {
            robot = new RobotK(hardwareMap);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

    }

    @Override
    public void run() {
        super.run();
        robot.update();

        if (gamepad1.right_bumper) { // replace this with a button for sprint
            robot.drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
        } else {
            robot.drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
        }

        if(gamepad1.b){
            robot.lift.retract(0.5);
        }

        if(gamepad1.a && !robot.lift.extended) {
            schedule(new ScoreSequence(ScoreSequence.OpModeType.TELEOP, robot));
        }


    }
}
