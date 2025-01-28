package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.CountDownTimer;
import org.firstinspires.ftc.teamcode.Utilities.FileUtilities;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;
import org.firstinspires.ftc.teamcode.teleop.QualTeleop;

public abstract class RI3WAbstractAutonomous extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();
    public RI3WHardware robot = new RI3WHardware();
    public ImprovedGamepad gamepad;
    QualTeleop.ClawState currentClawState = QualTeleop.ClawState.CLOSED;
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

        robot.frontLeft.setPower(AutoConfig.getInstance().speed);
        robot.frontRight.setPower(AutoConfig.getInstance().speed);
        robot.backLeft.setPower(AutoConfig.getInstance().speed);
        robot.backRight.setPower(AutoConfig.getInstance().speed);

        while (opModeIsActive() && (robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
                robot.backLeft.isBusy() && robot.backRight.isBusy())) {
            telemetryLog(robot.frontLeft);
            telemetryLog(robot.frontRight);
            telemetryLog(robot.backLeft);
            telemetryLog(robot.backRight);
        }

        double end_angle = robot.getAngle();

//        if(Math.abs(original_angle - end_angle) > 2){
//            turnToAngle(original_angle);
//        }

        telemetry.addData("Original Angle", original_angle);
        telemetry.addData("End Angle", end_angle);

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
        telemetry.addData("Target Position - fL", robot.frontLeft.getTargetPosition());
        telemetry.addData("Target Position - fR", robot.frontRight.getTargetPosition());
        telemetry.addData("Target Position - bL", robot.backLeft.getTargetPosition());
        telemetry.addData("Target Position - bR", robot.backRight.getTargetPosition());
        telemetry.addData("Current Position - fL", robot.frontLeft.getCurrentPosition());
        telemetry.addData("Current Position - fR", robot.frontRight.getCurrentPosition());
        telemetry.addData("Current Position - bL", robot.backLeft.getCurrentPosition());
        telemetry.addData("Current Position - bR", robot.backRight.getCurrentPosition());
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
        telemetry.addLine("Press X to continue.");
        telemetry.update();
        while(opModeIsActive() && !gamepad1.x){

        }
    }
    public enum SlidePosition
    {
        lowChamber,
        highChamber,
        lowBasket,
        highBasket,
        linearSlidesBase,
        lower
    }
    public void moveSlides(SlidePosition position){
        if(position == SlidePosition.lowChamber){
            robot.slidesV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slidesV.setTargetPosition(RI3WHardware.lowChamber);
            robot.slidesV.setPower(RI3WHardware.linearSlidesPower);
            while(opModeIsActive() && (Math.abs(robot.slidesV.getCurrentPosition()-RI3WHardware.lowChamber) > 100)){
                idle();
            }
        }
        else if (position == SlidePosition.highChamber) {
            robot.slidesV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slidesV.setTargetPosition(RI3WHardware.highChamber);
            robot.slidesV.setPower(RI3WHardware.linearSlidesPower);
            while(opModeIsActive() && (Math.abs(robot.slidesV.getCurrentPosition()-RI3WHardware.highChamber) > 100)){
                telemetry.addData("Distance from target3:", Math.abs(robot.slidesV.getCurrentPosition()-RI3WHardware.highChamber));
                telemetry.update();
                idle();
            }
        }
        else if (position == SlidePosition.lowBasket) {
            robot.slidesV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slidesV.setTargetPosition(RI3WHardware.lowBasket);
            robot.slidesV.setPower(RI3WHardware.linearSlidesPower);
            while(opModeIsActive() && (Math.abs(robot.slidesV.getCurrentPosition()-RI3WHardware.lowBasket) > 100)){
                idle();
            }
        }
        else if (position == SlidePosition.highBasket) {
            robot.slidesV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slidesV.setTargetPosition(RI3WHardware.highBasket);
            robot.slidesV.setPower(RI3WHardware.linearSlidesPower);
            while(opModeIsActive() && (Math.abs(robot.slidesV.getCurrentPosition()-RI3WHardware.highBasket) > 100)){
                idle();
            }
        }
        else if (position == SlidePosition.linearSlidesBase) {
            robot.slidesV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slidesV.setTargetPosition(RI3WHardware.linearSlidesBase);
            robot.slidesV.setPower(RI3WHardware.linearSlidesPower);
            while(opModeIsActive() && (Math.abs(robot.slidesV.getCurrentPosition()-RI3WHardware.linearSlidesBase) > 100)){
                telemetry.addData("Distance from target:", Math.abs(robot.slidesV.getCurrentPosition()-RI3WHardware.linearSlidesPower));
                telemetry.update();
                idle();
            }
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
                            moveSlides(SlidePosition.linearSlidesBase);
                            break;
                        default:
                            telemetry.addLine("Invalid Path: unexpected position " + components[1]);
                            break;
                    }
                } break;
                case "Slide":{
                    double slidesH_position = robot.SLIDESH_MIN;
                    switch(components[1]){
                        case "extend":
                            if (Double.parseDouble(components[2]) < robot.SLIDESH_MAX) {
                                slidesH_position = Double.parseDouble(components[2]);
                            }
                            break;
                        case "retract":
                            if (Double.parseDouble(components[2]) > robot.SLIDESH_MIN){
                                slidesH_position = Double.parseDouble(components[2]);
                            }
                            break;
                    }
                    robot.slidesH.setPosition(slidesH_position);
                    waitUntil(1000);

                } break;
                case "Claw":{
                    switch (components[1]){
                        case "open":
                            robot.openClaw();
                            waitUntil(500);
                            break;
                        case "close":
                            robot.closeClaw();
                            waitUntil(500);
                            break;
                    }
                } break;
                case "Lower" :{
                    int lowerValue = Integer.parseInt(components[1]);
                    robot.slidesV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.slidesV.setTargetPosition(robot.slidesV.getCurrentPosition() - lowerValue);
                    robot.slidesV.setPower(RI3WHardware.linearSlidesPower);
                    while(opModeIsActive() && (Math.abs(robot.slidesV.getCurrentPosition()-robot.slidesV.getTargetPosition()) > 50)){
                        idle();
                    }
                }break;
            }
            if (AutoConfig.getInstance().debugMode){
                waitForContinue();
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
    public void help(){
        telemetry.addLine("Set Up Mode");
        telemetry.addLine();
        telemetry.addData("startLocation", AutoConfig.getInstance().whereToStart);
        telemetry.addLine("LB: Staring on left");
        telemetry.addLine("RB: Staring on right");
        telemetry.addLine();
        telemetry.addData("parkValue", AutoConfig.getInstance().whereToPark);
        telemetry.addLine("X: Park edge");
        telemetry.addLine("B: Park corner");
        telemetry.addLine();
        telemetry.addData("Current Speed", AutoConfig.getInstance().speed);
        telemetry.addLine("Y: Increase Speed (0.1)");
        telemetry.addLine("A: Decrease Speed (0.1)");
        telemetry.addLine();
        telemetry.addData("delayNumSec", AutoConfig.getInstance().delayNumSec);
        telemetry.addLine("Dpad Up: Increase delay (1 sec)");
        telemetry.addLine("Dpad Down: Decrease delay (1 sec)");
        telemetry.addLine();
        telemetry.addData("Debug", AutoConfig.getInstance().debugMode);
        telemetry.addLine("Start: Enter debugMode");

        telemetry.update();
    }
    public void setUp() {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");
        FileUtilities.telemetry = telemetry;
        FileUtilities.readAutoConfig();

        while (!isStarted()){
            help();
            gamepad.update();

            if (gamepad.b.isPressed()){
                AutoConfig.getInstance().whereToPark = ParkPosition.CORNER;
            }

            if (gamepad.x.isPressed()){
                AutoConfig.getInstance().whereToPark = ParkPosition.EDGE;
            }

            if (gamepad.y.isInitialPress()){
                AutoConfig.getInstance().speed = AutoConfig.getInstance().speed + 0.05 ;
            }

            if (gamepad.a.isInitialPress()){
                AutoConfig.getInstance().speed = AutoConfig.getInstance().speed - 0.05 ;
            }

            if (gamepad.dpad_up.isInitialPress()){
                AutoConfig.getInstance().delayNumSec = AutoConfig.getInstance().delayNumSec + 1 ;
            }

            if (gamepad.dpad_down.isInitialPress()){
                AutoConfig.getInstance().delayNumSec = AutoConfig.getInstance().delayNumSec - 1 ;
            }

            if (gamepad.left_bumper.isInitialPress()){
                AutoConfig.getInstance().whereToStart = StartingPosition.LEFT;
            }

            if (gamepad.right_bumper.isInitialPress()){
                AutoConfig.getInstance().whereToStart = StartingPosition.RIGHT;
            }
            telemetry.update();
            if (gamepad.start.isInitialPress()){
                AutoConfig x = AutoConfig.getInstance();
                x.debugMode = !x.debugMode;
            }
        }
        FileUtilities.writeAutoConfig();
    }
    public void waitForDelay() {
        CountDownTimer cdt = new CountDownTimer(ElapsedTime.Resolution.SECONDS);
        cdt.setTargetTime(AutoConfig.getInstance().delayNumSec);
        while(cdt.hasRemainingTime() && !isStopRequested()){
            idle();
        }
    }
    public void waitUntil(int milliseconds) {
        CountDownTimer cdt = new CountDownTimer(ElapsedTime.Resolution.MILLISECONDS);
        cdt.setTargetTime(milliseconds);
        while(cdt.hasRemainingTime() && !isStopRequested()){
            idle();
        }
    }
}
