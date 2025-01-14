package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Vision.QualVisionProcessor;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

@TeleOp(name = "QualTeleop")
public class QualTeleop extends OpMode {

    public RI3WHardware robot = new RI3WHardware();
    double turningPower = 0;
    public ImprovedGamepad gamepad_1;
    private ImprovedGamepad gamepad_2;

    enum TeleopState{
        DRIVER_CONTROL,
        CENTERING
    }
    public enum ClawState{
        CLOSED,
        PRE_ClOSED
    }
    Double targetTurnAngle;
    int[] slidesV_position = {RI3WHardware.linearSlidesBase, RI3WHardware.lowChamber, RI3WHardware.highChamber, RI3WHardware.lowBasket, RI3WHardware.highBasket};
    TeleopState currentState = TeleopState.DRIVER_CONTROL;
    ClawState currentClawState = ClawState.CLOSED;
    int slidesVPositionCurrent = 0;
    double slidesH_position = 0;
    @Override
    public void init() {
        gamepad_1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad1");
        gamepad_2 = new ImprovedGamepad(gamepad2, new ElapsedTime(), "Gamepad2");

        // Update our telemetry to use FTC Dashboard
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        robot.init(this.hardwareMap, telemetry, true);
    }
    // for config
    public void init_loop() {
        gamepad_1.update();
        gamepad_2.update();
        init_help();

        if (gamepad_1.x.isInitialRelease()) {
            robot.setTeamColor(QualVisionProcessor.SampleColor.BLUE);
        }

        if (gamepad_1.b.isInitialRelease()) {
            robot.setTeamColor(QualVisionProcessor.SampleColor.RED);
        }
    }

    private void init_help() {
        telemetry.addData("Team Color", robot.getTeamColor());
        telemetry.addLine("Team color: X=blue, Y=red");
        help();
        telemetry.update();
    }
    private void help() {
        telemetry.addLine("Gamepad 1 Controls");
        telemetry.addLine("- Left stick= Direction of robot");
        telemetry.addLine("- Right stick= Turns robot");
        telemetry.addLine("- X = turn R +180");
        telemetry.addLine("Gamepad 2 Controls");
        telemetry.addLine("- Claw: A=close, B=open");
        telemetry.addLine("- slideH: RT=retract, RB=extend");
        telemetry.addLine("- slideV: LT=lower, LB=raise");
        telemetry.addLine("- Y= reset v slides encoders");
        telemetry.addLine("- dpad up/dn= moves v slides up/dn");
        telemetry.addLine("------------------");

    }

    @Override
    public void loop() {
        gamepad_1.update();
        gamepad_2.update();

        //This turns the robot relative 180 degrees
        if (gamepad_1.x.isInitialPress()) {
            targetTurnAngle = robot.getAngle() + 180;
        }

        Double turnToAngleSpeed = robot.getTurnToAngleSpeed(targetTurnAngle);

        if (turnToAngleSpeed != null && turnToAngleSpeed == 0) {
            targetTurnAngle = null;
            turnToAngleSpeed = null;
        }

        if(gamepad_1.right_stick.x.getValue() != 0){
            turningPower = .75 * gamepad_1.right_stick.x.getValue();
            targetTurnAngle = null;
        }else {
            if (turnToAngleSpeed != null) {
                turningPower = -turnToAngleSpeed;
            } else {
                turningPower = 0;
            }
        }

        double y = 0.5 * gamepad_1.left_stick.y.getValue();
        double x = 0.5 * gamepad_1.left_stick.x.getValue();

        //Override in case set positions aren't working/ are set incorrectly
        if (gamepad_2.dpad_up.isPressed()) {
            robot.slidesV.setPower(RI3WHardware.linearSlidesPower + 0.2);
            robot.slidesV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.slidesV.setTargetPosition(robot.slidesV.getCurrentPosition());
            // Does not need touch sensor
        } else if (gamepad_2.dpad_down.isPressed() && (!robot.touchRight.isPressed() && !robot.touchLeft.isPressed())) {
            robot.slidesV.setPower(-RI3WHardware.linearSlidesPower - 0.2);
            robot.slidesV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.slidesV.setTargetPosition(robot.slidesV.getCurrentPosition());
        } else if(gamepad_2.left_trigger.isInitialPress() && slidesVPositionCurrent > 0){
            //down
            robot.slidesV.setTargetPosition(slidesV_position[--slidesVPositionCurrent]);
            robot.slidesV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slidesV.setPower(RI3WHardware.linearSlidesPower + 0.2);
        } else if(gamepad_2.left_bumper.isInitialPress() && slidesVPositionCurrent < slidesV_position.length-1){
            //up
            robot.slidesV.setTargetPosition(slidesV_position[++slidesVPositionCurrent]);
            robot.slidesV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slidesV.setPower(RI3WHardware.linearSlidesPower + 0.2);
        } else {
            robot.slidesV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slidesV.setPower(RI3WHardware.linearSlidesPower + 0.2);
        }


        if(gamepad_2.y.isInitialPress()){
            robot.slidesV.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.slidesV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad_2.b.isInitialPress()) {
            robot.openClaw();
        } else if (gamepad_2.a.isInitialPress()) {
            currentClawState = ClawState.PRE_ClOSED;
            robot.slidesV.setTargetPosition(0);
            robot.slidesV.setPower(RI3WHardware.linearSlidesPower);
        }
        if(robot.slidesV.getCurrentPosition() == 0 && currentClawState == ClawState.PRE_ClOSED){
            currentClawState = ClawState.CLOSED;
            robot.closeClaw();
            robot.slidesV.setTargetPosition(slidesV_position[0]);
        }

        if(gamepad_2.right_trigger.isPressed()){

            telemetry.addLine("right trigger pushed");
            if (slidesH_position > robot.SLIDESH_MIN) {
                // retract
                slidesH_position -= 0.007;
                robot.slidesH.setPosition(slidesH_position);
            }
        }
        if(gamepad_2.right_bumper.isPressed()){
            telemetry.addLine("right bumper pushed");
            if (slidesH_position < robot.SLIDESH_MAX) {
                //extend
                slidesH_position += 0.007;
                robot.slidesH.setPosition(slidesH_position);
            }
        }

        if (gamepad_1.start.isInitialPress()) {
            currentState = TeleopState.CENTERING;
            robot.timer.reset();
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
            currentState = TeleopState.DRIVER_CONTROL;
        }

        if (currentState == TeleopState.DRIVER_CONTROL || (x > 0) || (y > 0) || (turningPower > 0)){
        } else if (currentState == TeleopState.CENTERING){
            x = robot.getPos();
        }
        robot.frontLeft.setPower(y + x + turningPower);
        robot.frontRight.setPower(y - x - turningPower);
        robot.backLeft.setPower(y - x + turningPower);
        robot.backRight.setPower(y + x - turningPower);

        help();
        telemetry.addData("Team Color", robot.getTeamColor());
        telemetry.addData("Teleop State", currentState);
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
        telemetry.addData("slideH Position", slidesH_position);
        telemetry.addData("slideV Current Index", slidesVPositionCurrent);
        telemetry.addData("slideV Position", robot.slidesV.getCurrentPosition());
        telemetry.addData("slideV Target Pos", robot.slidesV.getTargetPosition());

        // This doesn't work rn
        if (robot.getDetectedSample() != null){
            telemetry.addData("Best Sample", robot.getDetectedSample().toString());
        } else {
            telemetry.addLine("No detected sample.");
        }

        telemetry.addData("power", robot.power);
        telemetry.addData("error", robot.error);
        telemetry.addData("errorSum", robot.errorSum);
        telemetry.update();
    }
    public void turnRelative(double relativeAngle){
        double targetAngle = (robot.getAngle()+relativeAngle);
        turnToAngle(targetAngle);
    }

    private void turnToAngle(double targetAngle) {
    }

}
