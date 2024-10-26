package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.autonomous.RI3WAbstractAutonomous;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

public class servoTest extends RI3WAbstractAutonomous {
    public RI3WHardware robot = new RI3WHardware();
    public ImprovedGamepad gamepad;
    double set_position = 0;
    double position_max = 1.0;
    double position_min = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");
        robot.init(this.hardwareMap, telemetry);
        while(!isStarted()) {
            gamepad.update();
            if (gamepad.dpad_up.isInitialPress()) {
                set_position = set_position+0.1;
            }
            if (gamepad.dpad_down.isInitialPress()){
                set_position = set_position-0.1;
            }
            if (gamepad.dpad_right.isInitialPress()){
                set_position = set_position+0.01;
            }
            if(gamepad.dpad_left.isInitialPress()){
                set_position = set_position-0.01;
            }
            if (position_max < set_position){
                set_position = 0;
            }
            if (position_min > set_position){
                set_position = 0;
            }
            telemetry.addData("Position of Servo: ", set_position);
        }
        waitForStart();
        robot.claw.setPosition(set_position);
    }
}
