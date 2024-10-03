package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

public abstract class RI3WAbstractAutonomous extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();
    public RI3WHardware robot = new RI3WHardware();

    public void move(double yInches, double xInches) {
        int ticksY = (int) (yInches * robot.TICKS_PER_INCH);
        int ticksX = (int) (xInches * (robot.TICKS_PER_INCH/0.9));

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setTargetPosition(ticksY + ticksX);
        robot.frontRight.setTargetPosition(ticksY - ticksX);
        robot.backLeft.setTargetPosition(ticksY - ticksX);
        robot.backRight.setTargetPosition(ticksY + ticksX);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(robot.speed);
        robot.frontRight.setPower(robot.speed);
        robot.backLeft.setPower(robot.speed);
        robot.backRight.setPower(robot.speed);

        while (opModeIsActive() && (robot.frontLeft.isBusy() || robot.frontRight.isBusy() ||
                robot.backLeft.isBusy() || robot.backRight.isBusy())) {
            telemetryLog();
        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void telemetryLog() {
        telemetry.addData("angle", robot.getAngle());
        telemetry.update();
    }

    public void turnToAngle(double turnAngle) {

        //robot.getAngle is between -180 and 180, starting at 0
        double turnPower = 0;
        double targetAngle = AngleUnit.normalizeDegrees(turnAngle) + 180;
        double startAngle = robot.getAngle() + 180;
        double turnError = AngleUnit.normalizeDegrees(targetAngle - startAngle);
        while (opModeIsActive() && !(turnError < robot.turnTolerance && turnError > -robot.turnTolerance)) {
            if (turnError >= 0) {
                turnPower = turnError / 90;
                if (turnPower > robot.speed) {
                    turnPower = robot.speed;
                }
            } else if (turnError < 0) {
                turnPower = turnError / 90;
                if (turnPower < -robot.speed) {
                    turnPower = -robot.speed;
                }
            }
            robot.frontLeft.setPower(-turnPower);
            robot.frontRight.setPower(turnPower);
            robot.backLeft.setPower(-turnPower);
            robot.backRight.setPower(turnPower);

            double currentAngle = robot.getAngle() + 180;
            turnError = AngleUnit.normalizeDegrees(targetAngle - currentAngle);
            telemetryLog();
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
    }
    public void turnRelative(double realtiveAngle){
        double targetAngle = (robot.getAngle()+realtiveAngle);
        turnToAngle(targetAngle);
    }
    public void waitForContinue() {
        while (!gamepad1.x) {

        }

    }

}
