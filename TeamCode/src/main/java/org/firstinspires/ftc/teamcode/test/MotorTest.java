package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;

@TeleOp(name = "MotorTest", group = "test")
public class MotorTest extends OpMode {
    public RI3WHardware robot = new RI3WHardware();
    List<Map.Entry<String, DcMotor>> dcMotorList = new ArrayList<>();
    Integer activeIndex = (0);
    public ImprovedGamepad gamepad;

    double y;

    public void help() {
        telemetry.addLine("Use Dpad to choose motor");
        telemetry.addLine("Left stick Y");
        telemetry.addLine("           up - forward");
        telemetry.addLine("           dn - backward");
        telemetry.addLine("");
        telemetry.addData("current motor", dcMotorList.get(activeIndex).getKey());
        telemetry.addData("motor power", dcMotorList.get(activeIndex).getValue().getPower());
        telemetry.addData("encoder value", dcMotorList.get(activeIndex).getValue().getCurrentPosition());
        telemetry.addData("y stick", y);
    }
    
    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");
        robot.init(this.hardwareMap, telemetry);
        Set<Map.Entry<String, DcMotor>> dcMotorSet = this.hardwareMap.dcMotor.entrySet();
        dcMotorList.addAll(dcMotorSet);
    }

    @Override
    public void loop() {
        gamepad.update();

        dcMotorList.get(activeIndex).getValue().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(gamepad.dpad_up.isInitialPress()){
            dcMotorList.get(activeIndex).getValue().setPower(0);
        }
        if(gamepad.dpad_up.isInitialPress()){
            activeIndex++;
        }
        if(activeIndex == dcMotorList.size()){
            activeIndex = 0;
        }
        y = gamepad.left_stick.y.getValue();
        dcMotorList.get(activeIndex).getValue().setPower(y);

        help();
    }
}
