package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

@TeleOp(name = "RI3WTeleop")
public class RI3WTeleop extends OpMode {

    public RI3WHardware robot = new RI3WHardware();
    double turningPower = 0;
    public ImprovedGamepad gamepad;

    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");
        robot.init(this.hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        gamepad.update();

        if (gamepad.right_trigger.getValue() > 0) {
            turningPower = .3 * gamepad.right_trigger.getValue();
        } else if (gamepad.left_trigger.getValue() > 0) {
            turningPower = -.3 * gamepad.left_trigger.getValue();
        } else {
            turningPower = .75 * gamepad.right_stick.x.getValue();
        }

        double y = 1 * gamepad.left_stick.y.getValue();
        double x = 1 * gamepad.left_stick.x.getValue();
        double rx = turningPower;

        if (gamepad.dpad_up.isPressed()){
            robot.linearSlides.setPower(RI3WHardware.linearSlidesPower);
        } else if (gamepad.dpad_down.isPressed()) {
            robot.linearSlides.setPower(-RI3WHardware.linearSlidesPower);
        } else { robot.linearSlides.setPower(0);}

        robot.frontLeft.setPower(y + x + rx);
        robot.frontRight.setPower(y - x - rx);
        robot.backLeft.setPower(y - x + rx);
        robot.backRight.setPower(y + x - rx);

        telemetry.addData("frontLeft", robot.frontLeft.getCurrentPosition());
        telemetry.addData("frontRight", robot.frontRight.getCurrentPosition());
        telemetry.addData("backLeft", robot.backLeft.getCurrentPosition());
        telemetry.addData("backRight", robot.backRight.getCurrentPosition());
        telemetry.update();
    }
}
