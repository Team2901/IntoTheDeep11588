package org.firstinspires.ftc.teamcode.test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.autonomous.RI3WAbstractAutonomous;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;
@Autonomous(name = "testErrorSpeed", group = "test")
public class testErrorSpeed extends RI3WAbstractAutonomous {
    double increase_speed = (0.0);
    double set_distance_x = (0.0);
    double set_distance_y = (0.0);
    double x_max = (132);
    // only move up to 11 feet/132 inches
    double y_max = (132);
    double x_negative_max = (-132);
    double y_negative_max = (-132);
    public ImprovedGamepad gamepad;
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap, telemetry);
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");
        while(!isStarted()){
            gamepad.update();
            if(gamepad.dpad_up.isInitialPress()){
                if(increase_speed >= 1){
                    increase_speed = 0.0;
                }else{
                    increase_speed = increase_speed+0.1;
                }
            }
            if(gamepad.dpad_down.isInitialPress()){
                if(increase_speed <= -1){
                    increase_speed = 0.0;
                }else{
                    increase_speed = increase_speed-0.1;
                }
            }
            if(gamepad.dpad_right.isInitialPress()){
                if(increase_speed >= 1){
                    increase_speed = 0.0;
                }else{
                    increase_speed = increase_speed+0.01;
                }
            }
            if(gamepad.dpad_left.isInitialPress()){
                if(increase_speed <= -1){
                    increase_speed = 0.0;
                }else{
                    increase_speed = increase_speed-0.01;
                }
            }
            if (gamepad.y.isInitialPress()){
                if (set_distance_y <= y_negative_max){
                    set_distance_y = 0;
                }else{
                    set_distance_y = set_distance_y+5;
                }
            }
            if (gamepad.a.isInitialPress()){
                if(set_distance_y >= y_max){
                    set_distance_y = 0;
                }else {
                    set_distance_y = set_distance_y-5;
                }
            }
            if (gamepad.x.isInitialPress()){
                if(set_distance_x <= x_negative_max){
                    set_distance_x = 0;
                }else {
                    set_distance_x = set_distance_x - 5;
                }
            }
            if (gamepad.b.isInitialPress()){
                if(set_distance_x >= x_max){
                    set_distance_x = 0;
                }else {
                    set_distance_x = set_distance_x + 5;
                }
            }
            telemetry.addData( "increase_speed", increase_speed);
            telemetry.addData("set_distance_y", set_distance_y);
            telemetry.addData("set_distance_x", set_distance_x);
            telemetry.update();
        }
        waitForStart();
        robot.speed = increase_speed;
        move(set_distance_y, set_distance_x);
    }
}
