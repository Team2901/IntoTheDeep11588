package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

public abstract class RI3WAbstractAutonomous extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();
    public RI3WHardware robot = new RI3WHardware();
    public ImprovedGamepad gamepad;

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
        telemetry.addData("PIDFCoefficients", robot.frontLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("Target Position", robot.frontLeft.getTargetPosition());
        telemetry.addData("Current Position", robot.frontLeft.getCurrentPosition());
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
    //TODO Claw close and open, lift move to positions, other things????
    public void parsePath(String path) throws Exception {
        String[] pathSteps = path.split("\n");
        for (String step : pathSteps) {
            String[] components = step.split(" ");
            switch (components[0]) {
                case "Strafe": {
                    int sign;
                    switch (components[1]) {
                        case "right":
                            sign = 1;
                            break;
                        case "left":
                            sign = -1;
                            break;
                        default:
                            throw new Exception("Invalid path: unexpected direction " + components[1]);
                    }
                    double distanceValue = Double.parseDouble(components[2]);
                    switch (components[3]){
                        case "inches":
                            break;
                        case "centimeters":
                            distanceValue = distanceValue/2.54;
                            break;
                        default:
                            throw new Exception("Invalid path: unexpected unit " + components[3]);
                    }
                    move(0, distanceValue*sign);
                } break;
                case "Move": {
                    int sign;
                    switch (components[1]) {
                        case "forward":
                            sign = 1;
                            break;
                        case "back":
                            sign = -1;
                            break;
                        default:
                            throw new Exception("Invalid path: unexpected direction " + components[1]);
                    }
                    double distanceValue = Double.parseDouble(components[2]);
                    switch (components[3]){
                        case "inches":
                            break;
                        case "centimeters":
                            distanceValue = distanceValue/2.54;
                            break;
                        default:
                            throw new Exception("Invalid path: unexpected unit " + components[3]);
                    }
                    move(distanceValue*sign, 0);
                } break;
                case "Turn": {
                    int sign;
                    switch (components[1]){
                        case "clockwise":
                            sign = 1;
                            break;
                        case "counterclockwise":
                            sign = -1;
                            break;
                        default:
                            throw new Exception("Invalid turn: unexpected direction " + components[1]);
                    }
                    double turnValue= Double.parseDouble(components[2]);
                    switch (components[3]){
                        case "degrees":
                            break;
                        case "radians":
                            turnValue = turnValue*(180/Math.PI);
                            break;
                        default:
                            throw new Exception("Invalid turn: unexpected unit " + components[3]);
                    }
                    turnRelative(turnValue*sign);
                } break;
                /*
                case "Lift":{
                    switch(components[1]){
                        case "":
                            break;

                    }
                }
                 */
            }
        }
    }
}
