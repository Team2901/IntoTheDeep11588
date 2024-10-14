package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

@TeleOp(name = "RI6WTeleop")
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

        double y = 0.5 * gamepad.left_stick.y.getValue();
        double x = 0.5 * gamepad.left_stick.x.getValue();

        //This turns the robot relative 180 degrees
        if (gamepad.x.isInitialPress()) {
            turnRelative(180);
        }

        if (gamepad.dpad_up.isInitialPress()) {
            robot.linearSlides.setPower(RI3WHardware.linearSlidesPower);
            robot.linearSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (gamepad.dpad_down.isInitialPress()) {
            robot.linearSlides.setPower(-RI3WHardware.linearSlidesPower);
            robot.linearSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (gamepad.dpad_down.isInitialRelease() || gamepad.dpad_up.isInitialRelease()) {
            robot.linearSlides.setTargetPosition(robot.linearSlides.getCurrentPosition());
            robot.linearSlides.setPower(RI3WHardware.linearSlidesPower);
            robot.linearSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad.b.isInitialPress()) {
            robot.openClaw();
        } else if (gamepad.b.isInitialRelease()) {
            robot.closeClaw();
        }

        if (gamepad.left_bumper.isInitialPress()) {
            robot.runContExtake();
        } else if (gamepad.left_bumper.isInitialRelease()) {
            robot.stopContIntake();
        }

        if (gamepad.right_bumper.isInitialPress()) {
            robot.runContIntake();
        } else if (gamepad.right_bumper.isInitialRelease()) {
            robot.stopContIntake();
        }

        if (gamepad.a.isPressed()) {
            robot.arm.setPower(-.5);
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (gamepad.y.isPressed()) {
            robot.arm.setPower(.5);
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (gamepad.a.isInitialRelease() || gamepad.y.isInitialRelease()) {
            robot.arm.setTargetPosition(robot.arm.getCurrentPosition());
            robot.arm.setPower(.5);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //This is a soft-stop for the arm
        if (robot.arm.getCurrentPosition() > 0) {
            robot.arm.setTargetPosition(-10);
        } else if (robot.arm.getCurrentPosition() < -8500) {
            robot.arm.setTargetPosition(-8400);
        }

        robot.frontLeft.setPower(y + x + turningPower);
        robot.frontRight.setPower(y - x - turningPower);
        robot.backLeft.setPower(y - x + turningPower);
        robot.backRight.setPower(y + x - turningPower);

        telemetry.addData("frontLeft", robot.frontLeft.getCurrentPosition());
        telemetry.addData("frontRight", robot.frontRight.getCurrentPosition());
        telemetry.addData("backLeft", robot.backLeft.getCurrentPosition());
        telemetry.addData("backRight", robot.backRight.getCurrentPosition());
        telemetry.addData("Arm", robot.arm.getCurrentPosition());
        telemetry.addData("Claw Position", robot.claw.getPosition());
        telemetry.update();
    }
    public void turnRelative(double relativeAngle){
        double targetAngle = (robot.getAngle()+relativeAngle);
        turnToAngle(targetAngle);
    }

    private void turnToAngle(double targetAngle) {
    }

}
