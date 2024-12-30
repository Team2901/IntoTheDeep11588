package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

//tests servo
@TeleOp(name = "ServoTest", group = "test")
public class ServoTest extends OpMode {
    public RI3WHardware robot = new RI3WHardware();
    public ImprovedGamepad gamepad;
    double set_position = 0.5;
    double position_max = 1.0;
    double position_min = 0;

    public void help(){
        telemetry.addData("Position of Servo: ", set_position);
        telemetry.addLine("dpad.up = open");
        telemetry.addLine("dpad.down = close");
        telemetry.addLine("dpad.right = slowly open");
        telemetry.addLine("dpad.left = slowly close");
        telemetry.update();
    }
    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");
        robot.init(this.hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        gamepad.update();
        help();
        if (gamepad.dpad_up.isInitialPress()) {
            set_position = set_position+0.1;
        }
        if (gamepad.dpad_down.isInitialPress()){
            set_position = set_position-0.1;
        }
        if (gamepad.dpad_right.isInitialPress()){
            set_position = set_position+0.01;
        }
        if(gamepad.dpad_left.isInitialPress()){
            set_position = set_position-0.01;
        }
        if (position_max < set_position){
            set_position = 0;
        }
        if (position_min > set_position){
            set_position = 0;
        }
        robot.claw.setPosition(set_position);
    }
}
