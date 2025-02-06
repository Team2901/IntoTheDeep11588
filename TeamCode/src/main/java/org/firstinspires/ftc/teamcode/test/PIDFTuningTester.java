package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

import java.util.List;

public class PIDFTuningTester extends OpMode {
    public RI3WHardware robot = new RI3WHardware();
    public ImprovedGamepad gamepad;
    int coefficientIndex = 0;
    double[] coefficients = {5, 3, 0, 40};
    public void help(){
        telemetry.addLine("Use Dpad to choose coefficient");
        telemetry.addLine("Y = +10, A = -10, X = +1, b = -1");
        telemetry.addData("Current PID coefficients", robot.slidesV.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
    }
    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");
        robot.init(this.hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        gamepad.update();
        if (gamepad.dpad_down.isInitialPress() && coefficientIndex > 0){
            coefficientIndex--;
        }
        if(gamepad.dpad_up.isInitialPress() && coefficientIndex < 3){
            coefficientIndex++;
        }
        if (gamepad.y.isInitialPress()){
            coefficients[coefficientIndex] = coefficients[coefficientIndex]+1;
        } else if (gamepad.a.isInitialPress()){
            coefficients[coefficientIndex] = coefficients[coefficientIndex]-1;
        } else if (gamepad.x.isInitialPress()) {
            coefficients[coefficientIndex] = coefficients[coefficientIndex]+0.1;
        } else if (gamepad.b.isInitialPress()) {
            coefficients[coefficientIndex] = coefficients[coefficientIndex]-0.1;
        }
        robot.slidesV.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(coefficients[0], coefficients[1], coefficients[2], coefficients[3], MotorControlAlgorithm.PIDF));
        robot.slidesV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.slidesV.setTargetPosition(robot.slidesV.getCurrentPosition());
        robot.slidesV.setPower(gamepad.right_stick.y.getValue());
        help();
    }
}
