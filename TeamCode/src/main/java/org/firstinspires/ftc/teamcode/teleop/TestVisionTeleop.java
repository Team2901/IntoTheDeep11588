package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.DetectedSample;
import org.firstinspires.ftc.teamcode.Vision.QualVisionProcessor;
import org.firstinspires.ftc.teamcode.Vision.TrackedSample;
import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

@TeleOp(name = "TestVisionTeleop", group = "test")
@Config
public class TestVisionTeleop extends OpMode {

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

        // Initialize our robot
        robot.init(hardwareMap, telemetry, true);
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
        TrackedSample detectedSample = robot.getDetectedSample();
        telemetry.addData("Best Sample", detectedSample);
        // Tests if there is a sample present
        if(detectedSample != null){
            Point centroid = detectedSample.sample.centroid;
            // Converts centroid x position from pixels to screen percentage
            double cx = centroid.x / robot.getVisionPortalWidth();
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
