package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Bitmap;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.teamcode.Vision.QualVisionProcessor;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Point;

@TeleOp(name = "TestVisionTeleop", group = "test")
public class TestVisionTeleop extends OpMode {

    QualVisionProcessor testProcessor;
    VisionPortal portal;
    public RI3WHardware robot = new RI3WHardware();
    @Override
    public void init() {
        // Update our telemetry to use FTC Dashboard
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        testProcessor = new QualVisionProcessor(this.telemetry);
        portal = new VisionPortal.Builder()
                .addProcessor(testProcessor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        // Hook up the camera to FTC Dashboard
        //FtcDashboard.getInstance().startCameraStream(testProcessor, 0);

        // Initialize our robot
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        double power = 0;
        telemetry.addData("centroid Count", testProcessor.detectedSample);

        if(testProcessor.detectedSample != null){
            Point centroid = testProcessor.detectedSample.centroid;
            double cx = centroid.x / testProcessor.targetSize.width;
            double rx = 0.5;
            double zx = 0.10;
            double dx = 2 * (cx - rx); // value between -.5 -> .5
            double speedMod = 0.25;

            telemetry.addData("centroid", cx);
            telemetry.addData("direction", dx);
            if (Math.abs(dx) <= zx){
                power = 0;
            } else {
                power = speedMod * (dx - (Math.signum(dx) * zx));
            }

            robot.frontLeft.setPower(power);
            robot.frontRight.setPower(-power);
            robot.backLeft.setPower(-power);
            robot.backRight.setPower(power);
        } else {
            power = 0;
        }

        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(-power);
        robot.backLeft.setPower(-power);
        robot.backLeft.setPower(power);

        telemetry.addData("power", power);
        telemetry.update();
    }
}
