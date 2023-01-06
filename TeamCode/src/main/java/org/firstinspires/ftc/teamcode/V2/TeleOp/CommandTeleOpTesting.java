package org.firstinspires.ftc.teamcode.V2.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Vector2D;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.ScoringSystemV2EpicLift;

@TeleOp
public class CommandTeleOpTesting extends CommandOpMode {

    ScoringSystemV2EpicLift score;
    MecDrive drive;
    ColorRangeSensor distance;
    
    ParallelCommandGroup commands = new ParallelCommandGroup();
    
    private volatile boolean autoLinkageFlag, grabFlag, manualFlag, changeStackFlag, liftBrokenMode, optionsFlag;
    
    @Override
    public void initialize() {

        //Initializing flags
        autoLinkageFlag = true;
        grabFlag = true;
        manualFlag = true;
        liftBrokenMode = false;
        optionsFlag = true;

        
        score = new ScoringSystemV2EpicLift(hardwareMap, telemetry);
        drive = new MecDrive(hardwareMap,false, telemetry);


        //score.setLinkagePositionLogistic(Constants.linkageDown, 500);
        score.setGrabberPosition(Constants.open - 0.15);

        distance = hardwareMap.get(ColorRangeSensor.class, "distance");


        //Color sensor gain values
        distance.setGain(180);


    }

    @Override
    public void run() {
        super.run();

        
        //Drive
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;

        if(Math.abs(leftStickX) > Math.abs(leftStickY)){
            leftStickY = 0;

        }else if(Math.abs(leftStickY) > Math.abs(leftStickX)){
            leftStickX = 0;

        }else{
            leftStickY = 0;
            leftStickX = 0;
        }

        double finalLeftStickX = leftStickX;
        double finalLeftStickY = leftStickY;

        if (gamepad1.right_bumper) {
            
            commands.addCommands(
                   new InstantCommand(() -> drive.setPower(new Vector2D(finalLeftStickX * Constants.SPRINT_LINEAR_MODIFIER, finalLeftStickY * Constants.SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.SPRINT_ROTATIONAL_MODIFIER, false)) 
            );
            
        } else if(score.isExtended()){
            
            commands.addCommands(
                    new InstantCommand(() -> drive.setPower(new Vector2D(finalLeftStickX * Constants.EXTENDED_LINEAR_MODIFIER, finalLeftStickY * Constants.EXTENDED_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.EXTENDED_ROTATIONAL_MODIFIER, false))
            );
            
        } else{
            commands.addCommands(
                    new InstantCommand(() -> drive.setPower(new Vector2D(finalLeftStickX * Constants.NORMAL_LINEAR_MODIFIER, finalLeftStickY * Constants.NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.NORMAL_ROTATIONAL_MODIFIER, false))
            );
            
        }



        //Lift to Height
        if(gamepad1.left_trigger > 0.1){
            //score.setPower(0.2);
            if(score.getScoringMode() != ScoringSystemV2EpicLift.ScoringMode.ULTRA && !liftBrokenMode) {
                

                if(score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.LOW) {
                    
                    commands.addCommands(
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> score.autoGoToPosition()),
                                    new InstantCommand(() -> score.setLinkagePosition(0.71))
                            )
                    );
                    
                } else {
                    score.setLinkagePosition(Constants.linkageScoreV2 - 0.05);

                    commands.addCommands(
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> score.autoGoToPosition()),
                                    new InstantCommand(() -> score.setLinkagePosition(Constants.linkageScoreV2 - 0.05))
                            )
                    );
                    
                }
                
                //Scoring for Ground Junctions (Ultra)
            }else if (score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.ULTRA){
                
                commands.addCommands(
                        new InstantCommand(() -> score.setLinkagePosition(0.15))
                );
                
            }

        }

        //Scoring feature
        if(gamepad1.right_trigger > 0.1){

            
            if(score.getScoringMode() != ScoringSystemV2EpicLift.ScoringMode.ULTRA) {
                
                commands.addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> score.setGrabberPosition(Constants.score)),
                                new WaitCommand(600),
                                new InstantCommand(() -> score.setLinkagePosition(Constants.linkageUpV2)),
                                new WaitCommand(70),
                                new InstantCommand(() -> score.setLinkageConeStack(true)),
                                new InstantCommand(() -> score.setGrabberPosition(Constants.open - 0.15)),
                                new InstantCommand(() -> score.setExtended(false)),
                                new InstantCommand(() -> score.moveToPosition(0, 0.5))
                                
                        )
                );



                //Reset for Ground Junctions (Ultra) 
            }else{

                commands.addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> score.setGrabberPosition(Constants.open - 0.15)),
                                new WaitCommand(700)
                                
                        )
                );
                
            }

            if (liftBrokenMode) {

                //TODO: see what this actually does
                commands.addCommands(
                        new SequentialCommandGroup(
                                new WaitCommand(2000),
                                new InstantCommand(() -> score.lowerConeStack()),
                                new InstantCommand(() -> autoLinkageFlag = true),
                                new InstantCommand(() -> grabFlag = true),
                                new WaitCommand(500)


                        )

                        
                );
                
            }else {

                commands.addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> score.lowerConeStack()),
                                new InstantCommand(() -> autoLinkageFlag = true),
                                new InstantCommand(() -> grabFlag = true),
                                new WaitCommand(500)


                        )


                );
                
            }
            
            //Automated Grab
        }else if((distance.getNormalizedColors().red > 0.80 || distance.getNormalizedColors().blue > 0.80) && autoLinkageFlag){


            if(score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.ULTRA){
                commands.addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> score.setGrabberPosition(Constants.grabbing)),
                                new InstantCommand(() -> grabFlag = false),
                                new WaitCommand(650),
                                new InstantCommand(() -> score.setLinkagePosition(0.25)),
                                new InstantCommand(() -> autoLinkageFlag = false)

                        )
                );
                
            }else if(liftBrokenMode){

                commands.addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> score.setGrabberPosition(Constants.grabbing)),
                                new InstantCommand(() -> grabFlag = false),
                                new WaitCommand(650),
                                new InstantCommand(() -> score.setLinkagePosition(0.54)),
                                new InstantCommand(() -> autoLinkageFlag = false)

                        )
                );
                
            }else {

                commands.addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> score.setGrabberPosition(Constants.grabbing)),
                                new InstantCommand(() -> grabFlag = false),
                                new WaitCommand(650),
                                new InstantCommand(() -> score.setLinkagePosition(Constants.linkageScoreV2 - 0.05)),
                                new InstantCommand(() -> autoLinkageFlag = false)

                        )
                );
                
            }
            

        }
        
        
        //More Stuff
        if((gamepad1.left_bumper || gamepad1.dpad_up || gamepad1.dpad_down) && changeStackFlag){

            //Raise linkage by height of a cone (max height of 5)
            if(gamepad1.left_bumper || gamepad1.dpad_up) {

                commands.addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> score.setConeStack(5)),
                                new InstantCommand(() -> score.setLinkageConeStack(false)),
                                new InstantCommand(() -> changeStackFlag = false)

                        )
                );

                //Lower linkage by height of a cone (min height of 1)
            }else if(gamepad1.dpad_down){

                new SequentialCommandGroup(
                        new InstantCommand(() -> score.lowerConeStack()),
                        new InstantCommand(() -> score.setLinkageConeStack(false)),
                        new InstantCommand(() -> changeStackFlag = false),
                        new WaitCommand(200)

                );

            }
            

        }
        if(!gamepad1.dpad_down && !gamepad1.dpad_up && !gamepad1.left_bumper){
            commands.addCommands(
                    new InstantCommand(() -> changeStackFlag = true)
            );
        }


        //Linkage up position
        if(gamepad1.left_stick_button){
            
            commands.addCommands(
                    new InstantCommand(() -> score.setLinkagePosition(Constants.linkageScoreV2 - 0.05))
            );
            

        }



        //Manual open and close grabber
        if(gamepad1.start && manualFlag){
            if(score.getGrabberPosition() != Constants.open - 0.15) {
                
                commands.addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> score.setGrabberPosition(Constants.open - 0.15)),
                                new WaitCommand(300),
                                new InstantCommand(() -> grabFlag = true),
                                new InstantCommand(() -> manualFlag = false)
                        )
                );
                
                
            }else{

                commands.addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> score.setGrabberPosition(Constants.grabbing)),
                                new WaitCommand(300),
                                new InstantCommand(() -> grabFlag = false),
                                new InstantCommand(() -> manualFlag = false)
                        )
                );
                
            }
            
        }
        
        if(!gamepad1.start){
            commands.addCommands(
                    new InstantCommand(() -> manualFlag = true)
            );
        }


        //TODO: test these cuz not in parallel
        //Changing scoring modes (toggle)
        if(gamepad1.y){
            
            schedule(
                    new InstantCommand(() -> score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.LOW))
            );
            

        }else if(gamepad1.x){

            schedule(
                    new InstantCommand(() -> score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.MEDIUM))
            );
            
        }else if(gamepad1.b){

            schedule(
                    new InstantCommand(() -> score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.HIGH))
            );

        }else if(gamepad1.a){
            //Ground Junction
            schedule(
                    new InstantCommand(() -> score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.ULTRA))
            );
        }


        //Manual slides (dpad right and left)
        if(gamepad1.dpad_right){
            commands.addCommands(
                    new InstantCommand(() -> score.setPower(0.85))
            );
        }else if(gamepad1.dpad_left){
            commands.addCommands(
                    new InstantCommand(() -> score.setPower(-0.45))
            );

        }

        //TODO: check this cuz also not parallel
        if (gamepad1.ps && optionsFlag) {
            
            schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> optionsFlag = false),
                            new InstantCommand(() -> liftBrokenMode = !liftBrokenMode)
                    )
                    
            );
            
        }
        if (!gamepad1.ps) {
            schedule(
                    new InstantCommand(() -> optionsFlag = true)

            );
        }

        if (gamepad2.dpad_up) {
            commands.addCommands(
                    new InstantCommand(() -> score.setLinkagePosition(score.getLeftLinkage() + 0.001))
            );
            
        }

        if (gamepad2.dpad_down) {
            commands.addCommands(
                    new InstantCommand(() -> score.setLinkagePosition(score.getLeftLinkage() - 0.001))
            );       
        }





        //FeedForward or not
        if(score.isExtended() && !gamepad1.dpad_left && !gamepad1.dpad_right){

            commands.addCommands(
                    new InstantCommand(() -> score.setPowerSingular(0.23))
            );

        }else if(!score.isExtended() && !gamepad1.dpad_left && !gamepad1.dpad_right){
            commands.addCommands(
                    new InstantCommand(() -> score.setPowerSingular(0))
            );
        }
        
        
        
        
        
        
        
        schedule(commands);
        
        
    }
    
    
}
