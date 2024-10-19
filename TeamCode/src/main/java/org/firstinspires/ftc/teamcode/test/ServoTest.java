package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;
// TODO working on this, haven't added/edited anything yet
@TeleOp(name = "ServoTest", group = "test")
public class ServoTest extends OpMode {
    public RI3WHardware robot = new RI3WHardware();
    DcMotorEx[] motorArray;
    String[] motorNames;
    Integer activeIndex = (0);
    public ImprovedGamepad gamepad;
    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");
        robot.init(this.hardwareMap, telemetry);
        motorArray = new DcMotorEx[] {robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight, robot.linearSlides, robot.arm};
        motorNames = new String[] {"frontLeft", "frontRight", "backLeft", "backRight", "linearSlide", "arm"};
    }

    @Override
    public void loop() {
        gamepad.update();
        //robot.frontRight.getDeviceName();
        if(gamepad.dpad_up.isInitialPress()){
            motorArray[activeIndex].setPower(0);
        }
        if(gamepad.dpad_up.isInitialPress()){
            activeIndex++;
        }
        if(activeIndex == 6){
            activeIndex = 0;
        }

        motorArray[activeIndex].setPower(gamepad.left_stick.y.getValue());

        telemetry.addData("current motor", motorNames[activeIndex]);
        telemetry.addData("motor power", motorArray[activeIndex].getPower());
        telemetry.addData("encoder value", motorArray[activeIndex].getCurrentPosition());
    }
}
