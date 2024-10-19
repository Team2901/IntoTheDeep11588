package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utilities.CountDownTimer;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

public abstract class RI3WAbstractAutonomous extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();
    public RI3WHardware robot = new RI3WHardware();
    public ImprovedGamepad gamepad;
    int delayNumSec = 0;
    public void move(double yInches, double xInches) {
        double original_angle = robot.getAngle();

        int ticksY = (int) (yInches * RI3WHardware.TICKS_PER_INCH);
        int ticksX = (int) (xInches * (RI3WHardware.TICKS_PER_INCH /0.9));

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
            telemetryLog(robot.frontLeft);
        }

        /*double end_angle = robot.getAngle();

        if(Math.abs(original_angle - end_angle) > 2){
            turnToAngle(original_angle);
        }*/

        telemetry.addData("Original Angle", original_angle);
        //telemetry.addData("End Angle", end_angle);

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void telemetryLog(DcMotorEx dcMotorEx) {
        telemetry.addData("angle", robot.getAngle());
        telemetry.addData("PIDFCoefficients", dcMotorEx.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("Target Position", dcMotorEx.getTargetPosition());
        telemetry.addData("Current Position", dcMotorEx.getCurrentPosition());
        telemetry.update();
    }

    public void turnToAngle(double turnAngle) {

        //robot.getAngle is between -180 and 180, starting at 0
        double turnPower = robot.getTurnToAngleSpeed(turnAngle);
        while (opModeIsActive() && turnPower != 0) {
            robot.frontLeft.setPower(-turnPower);
            robot.frontRight.setPower(turnPower);
            robot.backLeft.setPower(-turnPower);
            robot.backRight.setPower(turnPower);

            turnPower = robot.getTurnToAngleSpeed(turnAngle);
            telemetryLog(robot.frontLeft);
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
    }
    public void turnRelative(double relativeAngle){
        double targetAngle = (robot.getAngle()+relativeAngle);
        turnToAngle(targetAngle);
    }
    public void waitForContinue() {
        while (!gamepad1.x) {

        }

    }
    public enum SlidePosition
    {
        lowChamber,
        highChamber,
        lowBasket,
        highBasket,
        base
    }
    public void moveSlides(SlidePosition position){
        if(position == SlidePosition.lowChamber){
            robot.PIDLoop(RI3WHardware.lowChamber);
        }
        else if (position == SlidePosition.highChamber) {
            robot.PIDLoop(RI3WHardware.highChamber);
        }
        else if (position == SlidePosition.lowBasket) {
            robot.PIDLoop(RI3WHardware.lowBasket);
        }
        else if (position == SlidePosition.highBasket) {
            robot.PIDLoop(RI3WHardware.highBasket);
        }
        else if (position == SlidePosition.base) {
            robot.PIDLoop(RI3WHardware.linearSlidesBase);
        }else{
            telemetry.addLine("Invalid Position: "+position);
        }
    }
    //TODO Claw close and open, other things????
    public void parsePath(String path) {
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
                            telemetry.addLine("Invalid path: unexpected direction " + components[1]);
                            return;
                    }
                    double distanceValue = Double.parseDouble(components[2]);
                    switch (components[3]){
                        case "inches":
                            break;
                        case "centimeters":
                            distanceValue = distanceValue/2.54;
                            break;
                        default:
                            telemetry.addLine("Invalid path: unexpected unit " + components[3]);
                            return;
                    }
                    move(0, distanceValue*sign);
                } break;
                case "Move": {
                    int sign;
                    double offset = 0;
                    switch (components[1]) {
                        case "forward":
                            telemetry.addLine("Planning to move forward");
                            sign = 1;
                            break;
                        case "back":
                        case "backward":
                        case "backwards":
                            sign = -1;
                            break;
                        default:
                            telemetry.addLine("Invalid path: unexpected direction " + components[1]);
                            return;
                    }
                    double distanceValue = Double.parseDouble(components[2]);
                    switch (components[3]){
                        case "inches":
                            telemetry.addData("Inches:", distanceValue);
                            break;
                        case "centimeters":
                            distanceValue = distanceValue/2.54;
                            break;
                        default:
                            telemetry.addLine("Invalid path: unexpected unit " + components[3]);
                            return;
                    }
                    if(components.length >= 5){
                        switch (components[4]){
                            case "clawOffset":
                                offset = RI3WHardware.clawOffset;
                                break;
                            case "":
                                break;
                            default:
                                telemetry.addLine("Invalid path: unexpected offset"+ components[4]);
                                return;
                        }
                    }
                    telemetry.addLine("Execute move");
                    telemetry.update();
                    move((distanceValue*sign)+offset, 0);
                } break;
                case "TurnR": {
                    int sign;
                    switch (components[1]){
                        case "clockwise":
                            sign = 1;
                            break;
                        case "counterclockwise":
                            sign = -1;
                            break;
                        default:
                            telemetry.addLine("Invalid turn: unexpected direction " + components[1]);
                            return;
                    }
                    double turnValue= Double.parseDouble(components[2]);
                    switch (components[3]){
                        case "degrees":
                            break;
                        case "radians":
                            turnValue = turnValue*(180/Math.PI);
                            break;
                        default:
                            telemetry.addLine("Invalid turn: unexpected unit " + components[3]);
                            return;
                    }
                    turnRelative(turnValue*sign);
                } break;
                case "TurnA": {
                    int sign;
                    switch (components[1]) {
                        case "clockwise":
                            sign = 1;
                            break;
                        case "counterclockwise":
                            sign = -1;
                            break;
                        default:
                            telemetry.addLine("Invalid turn: unexpected direction " + components[1]);
                            return;
                    }
                    double turnValue = Double.parseDouble(components[2]);
                    switch (components[3]) {
                        case "degrees":
                            break;
                        case "radians":
                            turnValue = turnValue * (180 / Math.PI);
                            break;
                        default:
                            telemetry.addLine("Invalid turn: unexpected unit " + components[3]);
                            return;
                    }
                    turnToAngle(turnValue * sign);
                } break;
                case "Lift":{
                    switch(components[1]){
                        case "lowBasket":
                            moveSlides(SlidePosition.lowBasket);
                            break;
                        case "highBasket":
                            moveSlides(SlidePosition.highBasket);
                            break;
                        case "lowChamber":
                            moveSlides(SlidePosition.lowChamber);
                            break;
                        case "highChamber":
                            moveSlides(SlidePosition.highChamber);
                            break;
                        case "base":
                            moveSlides(SlidePosition.base);
                            break;
                        default:
                            telemetry.addLine("Invalid Path: unexpected position " + components[1]);
                            break;
                    }
                } break;
            }
        }
    }

    enum ParkPosition {
        CORNER,
        EDGE
    }

    enum StartingPosition {
        LEFT,
        RIGHT
    }

    ParkPosition whereToPark = ParkPosition.CORNER;
    StartingPosition whereToStart = StartingPosition.LEFT;
    public void help(){
        telemetry.addLine("Set Up Mode");
        telemetry.addLine();
        telemetry.addData("startLocation", whereToStart);
        telemetry.addLine("LB: Staring on left");
        telemetry.addLine("RB: Staring on right");
        telemetry.addLine();
        telemetry.addData("parkValue", whereToPark);
        telemetry.addLine("X: Park edge");
        telemetry.addLine("B: Park corner");
        telemetry.addLine();
        telemetry.addData("Current Speed", robot.speed);
        telemetry.addLine("Y: Increase Speed (0.1)");
        telemetry.addLine("A: Decrease Speed (0.1)");
        telemetry.addLine();

        telemetry.update();
    }
    public void setUp() {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");

        while (!isStarted()){
            help();
            gamepad.update();

            if (gamepad.b.isPressed()){
                whereToPark = ParkPosition.CORNER;
            }

            if (gamepad.x.isPressed()){
                whereToPark = ParkPosition.EDGE;
            }

            if (gamepad.y.isInitialPress()){
                robot.speed = robot.speed + 0.05 ;
            }

            if (gamepad.a.isInitialPress()){
                robot.speed = robot.speed - 0.05 ;
            }

            if (gamepad.dpad_up.isInitialPress()){
                delayNumSec = delayNumSec + 1 ;
            }

            if (gamepad.dpad_down.isInitialPress()){
                delayNumSec = delayNumSec - 1 ;
            }

            if (gamepad.left_bumper.isInitialPress()){
                whereToStart = StartingPosition.LEFT;
            }

            if (gamepad.right_bumper.isInitialPress()){
                whereToStart = StartingPosition.RIGHT;
            }
            telemetry.update();
        }
    }
    private void waitForDelay() {
        CountDownTimer cdt = new CountDownTimer(ElapsedTime.Resolution.SECONDS);
        cdt.setTargetTime(delayNumSec);
        while(cdt.hasRemainingTime()){

        }
    }
}
