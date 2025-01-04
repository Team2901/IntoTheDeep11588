package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;

//tests servo
@TeleOp(name = "ServoTest", group = "test")
public class ServoTest extends OpMode {
    public RI3WHardware robot = new RI3WHardware();
    public ImprovedGamepad gamepad;
    List<Map.Entry<String, Servo>> servoList = new ArrayList<>();
    Integer activeIndex = (0);
    double set_position = 0.2;
    double position_max = 0.46;
    double position_min = 0.2;

    public void help(){
        telemetry.addLine("Use Dpad to choose motor");
        telemetry.addLine("dpad.up = next servo");
        telemetry.addData("current motor", servoList.get(activeIndex).getKey());
        telemetry.addData("Position of Servo: ", set_position);
        telemetry.addLine("y = open");
        telemetry.addLine("a = close");
        telemetry.addLine("x = slowly open");
        telemetry.addLine("b = slowly close");
        telemetry.update();
    }
    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");
        robot.init(this.hardwareMap, telemetry);
        Set<Map.Entry<String, Servo>> servoSet = this.hardwareMap.servo.entrySet();
        servoList.addAll(servoSet);
    }

    @Override
    public void loop() {
        gamepad.update();
        help();
        // Change to isPressed but need to do later
        if (gamepad.y.isInitialPress()) {
            set_position = set_position+0.1;
        }
        if (gamepad.a.isInitialPress()){
            set_position = set_position-0.1;
        }
        if (gamepad.x.isInitialPress()){
            set_position = set_position+0.01;
        }
        if(gamepad.b.isInitialPress()){
            set_position = set_position-0.01;
        }
        if (position_max < set_position){
            set_position = position_max;
        }
        if (position_min > set_position) {
            set_position = position_min;
        }
        if(gamepad.dpad_up.isInitialPress()){
            activeIndex++;
        }
        if(activeIndex == servoList.size()){
            activeIndex = 0;
        }
        servoList.get(activeIndex).getValue().setPosition(set_position);
    }
}
