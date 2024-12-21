package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Vision.QualVisionProcessor;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

@TeleOp(name = "RI6WTeleop v2")
public class RI3WTeleop extends OpMode {

    public RI3WHardware robot = new RI3WHardware();
    double turningPower = 0;
    public ImprovedGamepad gamepad_1;
    private ImprovedGamepad gamepad_2;

    Double targetTurnAngle;

    @Override
    public void init() {
        gamepad_1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad1");
        gamepad_2 = new ImprovedGamepad(gamepad2, new ElapsedTime(), "Gamepad2");
        robot.init(this.hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        gamepad_1.update();
        gamepad_2.update();


        if (gamepad_1.x.isInitialRelease()) {
            robot.setTeamColor(QualVisionProcessor.SampleColor.BLUE);
        }

        if (gamepad_1.b.isInitialRelease()) {
            robot.setTeamColor(QualVisionProcessor.SampleColor.RED);
        }

        //This turns the robot relative 180 degrees
        if (gamepad_2.x.isInitialPress()) {
            targetTurnAngle = robot.getAngle() + 180;
        }

        Double turnToAngleSpeed = robot.getTurnToAngleSpeed(targetTurnAngle);

        if (turnToAngleSpeed != null && turnToAngleSpeed == 0) {
            targetTurnAngle = null;
            turnToAngleSpeed = null;
        }

        if (gamepad_1.right_trigger.getValue() > 0) {
            turningPower = .3 * gamepad_1.right_trigger.getValue();
            targetTurnAngle = null;
        } else if (gamepad_1.left_trigger.getValue() > 0) {
            turningPower = -.3 * gamepad_1.left_trigger.getValue();
            targetTurnAngle = null;
        } else if (gamepad_1.right_stick.x.getValue() != 0){
            turningPower = -.75 * gamepad_1.right_stick.x.getValue();
            targetTurnAngle = null;
        } else {
            if (turnToAngleSpeed != null) {
                turningPower = -turnToAngleSpeed;
            } else {
                turningPower = 0;
            }
        }

        double y = 0.5 * gamepad_1.left_stick.y.getValue();
        double x = 0.5 * gamepad_1.left_stick.x.getValue();

        //TODO: isPressed
        if (gamepad_1.dpad_up.isInitialPress()) {
            robot.linearSlides.setPower(RI3WHardware.linearSlidesPower);
            robot.linearSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (gamepad_1.dpad_down.isInitialPress()) {
            robot.linearSlides.setPower(-RI3WHardware.linearSlidesPower);
            robot.linearSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (gamepad_1.dpad_down.isInitialRelease() || gamepad_1.dpad_up.isInitialRelease()) {
            robot.linearSlides.setTargetPosition(robot.linearSlides.getCurrentPosition());
            robot.linearSlides.setPower(RI3WHardware.linearSlidesPower);
            robot.linearSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad_2.b.isInitialPress()) {
            robot.openClaw();
        } else if (gamepad_2.b.isInitialRelease()) {
            robot.closeClaw();
        }

        if (gamepad_1.start.isInitialPress()) {
            // Start centering (initializing it)
            //
        } else if (gamepad_1.start.isPressed()) {
            // if slides lowering, do __
               // retract then lower
            // if centering, do:
               // set y = 0, turning power = 0, x = results from new routine (l84 - TVT) call
               // if power = 0 then change state to lowering
            // if extending, do __
               // 
            // if grabbing, do __
            // if retracting, do __
            // if done, do __
        } else if (gamepad_1.start.isInitialRelease()) {
            // cancels all auto centering/sample pick up
        } else


        /*
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

         */

        robot.frontLeft.setPower(y + x + turningPower);
        robot.frontRight.setPower(y - x - turningPower);
        robot.backLeft.setPower(y - x + turningPower);
        robot.backRight.setPower(y + x - turningPower);

        telemetry.addData("Team Color", robot.getTeamColor());
        telemetry.addData("Current Angle", robot.getAngle());
        telemetry.addData("Target Angle", this.targetTurnAngle);
        telemetry.addData("Target Turn Speed ", turnToAngleSpeed);
        telemetry.addLine();
        telemetry.addData("frontLeft", robot.frontLeft.getCurrentPosition());
        telemetry.addData("frontRight", robot.frontRight.getCurrentPosition());
        telemetry.addData("backLeft", robot.backLeft.getCurrentPosition());
        telemetry.addData("backRight", robot.backRight.getCurrentPosition());
        //telemetry.addData("Arm", robot.arm.getCurrentPosition());
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
