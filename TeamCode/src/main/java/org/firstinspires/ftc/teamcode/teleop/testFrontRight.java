package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

@TeleOp(name = "testFrontRight", group = "test")
public class testFrontRight extends OpMode {
    public RI3WHardware robot = new RI3WHardware();
    public ImprovedGamepad gamepad;
    double turningPower = 0;

    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");
        robot.init(this.hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        double y = 1 * gamepad.left_stick.y.getValue();
        double x = 1 * gamepad.left_stick.x.getValue();
        double rx = turningPower;

        robot.frontRight.setPower(y - x - rx);
    }
}
