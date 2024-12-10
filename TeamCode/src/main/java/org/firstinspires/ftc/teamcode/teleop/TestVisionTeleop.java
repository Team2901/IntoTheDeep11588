package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.QualVisionProcessor;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

@TeleOp(name = "TestVisionTeleop", group = "test")
@Config
public class TestVisionTeleop extends OpMode {


    QualVisionProcessor testProcessor;
    VisionPortal portal;
    public RI3WHardware robot = new RI3WHardware();
    // Slows down or speeds up based on distance from target
    public static double speedMod = 1;
    public static double Ki = 1;
    // Percent error allowed when approaching target
    public static double tolerance = 0.10;
    ElapsedTime timer = new ElapsedTime();
    double errorSum = 0;
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
        FtcDashboard.getInstance().startCameraStream(testProcessor, 0);

        // Initialize our robot
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        timer.reset();
    }

    @Override
    public void loop() {
        double power = 0;
        double error = 0;
        telemetry.addData("Best Sample", testProcessor.detectedSample);

        // Tests if there is a sample present
        if(testProcessor.detectedSample != null){
            Point centroid = testProcessor.detectedSample.sample.centroid;
            // Converts centroid x position from pixels to screen percentage
            double cx = centroid.x / testProcessor.targetSize.width;
            // Target center for the sample
            double tx = QualVisionProcessor.tx;
            // Distance from centroid x position to target center
            double dx = cx - tx;

            // If error withing the accepted range stop
            if(Math.abs(dx) <= tolerance){
                error = 0;
                errorSum = 0;
            }
            // Error is based on how far from accepted range
            else{
                error = dx - (Math.signum(dx) * tolerance);
            }
            // Integrate the error
            errorSum += error*timer.seconds();

            telemetry.addData("cx", cx);
            telemetry.addData("distance", dx);

            // Use PID to set power
            power = (speedMod * error) + (Ki * errorSum);

        } else {
            power = 0;
        }

        // Strafe left or right
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(-power);
        robot.backLeft.setPower(-power);
        robot.backRight.setPower(power);

        telemetry.addData("power", power);
        telemetry.addData("error", error);
        telemetry.addData("errorSum", errorSum);
        timer.reset();
    }
}
