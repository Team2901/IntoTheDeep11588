package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

@TeleOp(name = "PID TUNER")
public class PIDTuner extends OpMode {
    RI3WHardware robot;
    int goalPos = 0;
    ImprovedGamepad gamepad;
    double incrimentValue = .001;
    @Override
    public void init() {
        robot = new RI3WHardware();
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad");
    }

    @Override
    public void loop() {
        telemetry.addLine("A = increase Kg, B = decrease Kg, x = * 10, y = / 10, left bump = increase goal, right decrease");
        telemetry.addData("Kg", robot.Kg);
        telemetry.addData("Goal Position", goalPos);
        if (gamepad.a.isInitialPress()) {
            robot.Kg += incrimentValue;
        } else if (gamepad.b.isInitialPress()) {
            robot.Kg -= incrimentValue;
        }
        if (gamepad.x.isInitialPress()) {
            incrimentValue *= 10;
        } else if (gamepad.y.isInitialPress()) {
            incrimentValue /= 10;
        }

        if (gamepad.left_bumper.isInitialPress()) {
            goalPos += 50;
        } else if (gamepad.right_bumper.isInitialPress()) {
            goalPos -= 50;
        }
        robot.PIDLoop(goalPos);
        telemetry.update();
    }
}
