package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.autonomous.RI3WAbstractAutonomous;
@Autonomous(name = "testTurning", group = "test")
public class testTurning extends RI3WAbstractAutonomous {
    boolean relativeAbsoluteTurn = true; //True is relativeTurn false is absoluteTurn
    int desiredAngle = 0;
    public ImprovedGamepad gamepad;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap, telemetry);
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");
        while (!isStarted()) {
            gamepad.update();
            if(gamepad.dpad_up.isInitialPress()){
                if(desiredAngle >= 360){
                    desiredAngle = 0;
                }else{
                    desiredAngle = desiredAngle+10;
                }
            }
            if(gamepad.dpad_down.isInitialPress()){
                if(desiredAngle <= -360){
                    desiredAngle = 0;
                }else{
                    desiredAngle = desiredAngle-10;
                }
            }
            if(gamepad.dpad_left.isInitialPress()){
                if(desiredAngle <= -360){
                    desiredAngle = 0;
                }else{
                    desiredAngle = desiredAngle-1;
                }
            }
            if(gamepad.dpad_right.isInitialPress()){
                if(desiredAngle >= 360){
                    desiredAngle = 0;
                }else{
                    desiredAngle = desiredAngle+1;
                }
            }
            if(gamepad.a.isInitialPress()){
                relativeAbsoluteTurn = !relativeAbsoluteTurn;
            }
            telemetry.addData("relativeAbsoluteTurn", relativeAbsoluteTurn);
            telemetry.addData("desiredAngle", desiredAngle);
            telemetry.update();
        }
        waitForStart();
        if(relativeAbsoluteTurn == true){
            turnRelative(desiredAngle);
        }else{
            turnToAngle(desiredAngle);
        }
    }
}
