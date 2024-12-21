package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.autonomous.RI3WAbstractAutonomous;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

@TeleOp(name = "RI6WTeleop v2")
public class RI3WTeleop extends OpMode {

    public RI3WHardware robot = new RI3WHardware();
    double turningPower = 0;
    public ImprovedGamepad gamepad;


    Double targetTurnAngle;

    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");
        robot.init(this.hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        gamepad.update();


        //This turns the robot relative 180 degrees
        if (gamepad.x.isInitialPress()) {
            targetTurnAngle = robot.getAngle() + 180;
        }

        Double turnToAngleSpeed = robot.getTurnToAngleSpeed(targetTurnAngle);

        if (turnToAngleSpeed != null && turnToAngleSpeed == 0) {
            targetTurnAngle = null;
            turnToAngleSpeed = null;
        }

        if (gamepad.right_trigger.getValue() > 0) {
            turningPower = .3 * gamepad.right_trigger.getValue();
            targetTurnAngle = null;
        } else if (gamepad.left_trigger.getValue() > 0) {
            turningPower = -.3 * gamepad.left_trigger.getValue();
            targetTurnAngle = null;
        } else if (gamepad.right_stick.x.getValue() != 0){
            turningPower = -.75 * gamepad.right_stick.x.getValue();
            targetTurnAngle = null;
        } else {
            if (turnToAngleSpeed != null) {
                turningPower = -turnToAngleSpeed;
            } else {
                turningPower = 0;
            }
        }

        double y = 0.5 * gamepad.left_stick.y.getValue();
        double x = 0.5 * gamepad.left_stick.x.getValue();

        //TODO: isPressed
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

        if (gamepad.start.isInitialPress()) {
            // Start centering (initializing it)
            //
        } else if (gamepad.start.isPressed()) {
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
        } else if (gamepad.start.isInitialRelease()) {
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
