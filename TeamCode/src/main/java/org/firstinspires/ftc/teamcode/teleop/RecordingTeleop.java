package org.firstinspires.ftc.teamcode.teleop;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

import java.io.DataOutputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

@TeleOp(name = "RecordingTeleop")
public class RecordingTeleop extends OpMode {
    public RI3WHardware robot = new RI3WHardware();

    FileOutputStream fileOutputStream = new FileOutputStream(Environment.getExternalStorageDirectory().getAbsolutePath() + "/test.log", false);
    DataOutputStream writeFile = new DataOutputStream(fileOutputStream);
    double turningPower = 0;
    public ImprovedGamepad gamepad;
    ElapsedTime timer;

    public RecordingTeleop() throws FileNotFoundException {
    }

    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");
        robot.init(this.hardwareMap, telemetry);
        timer  = new ElapsedTime();
        timer.reset();
    }

    @Override
    public void loop() {
        gamepad.update();

        if (gamepad.right_trigger.getValue() > 0) {
            turningPower = .3 * gamepad.right_trigger.getValue();
        } else if (gamepad.left_trigger.getValue() > 0) {
            turningPower = -.3 * gamepad.left_trigger.getValue();
        } else {
            turningPower = .75 * gamepad.right_stick.x.getValue();
        }

        double y = 1 * gamepad.left_stick.y.getValue();
        double x = 1 * gamepad.left_stick.x.getValue();
        double rx = turningPower;

        robot.frontLeft.setPower(y + x + rx);
        robot.frontRight.setPower(y - x - rx);
        robot.backLeft.setPower(y - x + rx);
        robot.backRight.setPower(y + x - rx);

        try {
            recordInput();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public void recordInput() throws IOException {
        // rx ry lx ly a b x y up down left right r_bumper l_bumper r_trigger l_trigger
        writeFile.writeDouble(timer.milliseconds());
        writeFile.writeDouble(gamepad.right_stick.x.getValue());
        writeFile.writeDouble(gamepad.right_stick.y.getValue());
        writeFile.writeDouble(gamepad.left_stick.x.getValue());
        writeFile.writeDouble(gamepad.left_stick.y.getValue());
        writeFile.writeBoolean(gamepad.a.isPressed());
        writeFile.writeBoolean(gamepad.b.isPressed());
        writeFile.writeBoolean(gamepad.x.isPressed());
        writeFile.writeBoolean(gamepad.y.isPressed());
        writeFile.writeBoolean(gamepad.dpad_up.isPressed());
        writeFile.writeBoolean(gamepad.dpad_down.isPressed());
        writeFile.writeBoolean(gamepad.dpad_left.isPressed());
        writeFile.writeBoolean(gamepad.dpad_right.isPressed());
        writeFile.writeBoolean(gamepad.right_bumper.isPressed());
        writeFile.writeBoolean(gamepad.left_bumper.isPressed());
        writeFile.writeDouble(gamepad.right_trigger.getValue());
        writeFile.writeDouble(gamepad.left_trigger.getValue());
        telemetry.addData("Data was written", true);
        writeFile.flush();
    }
}
