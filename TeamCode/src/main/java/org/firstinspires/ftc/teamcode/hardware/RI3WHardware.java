package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utilities.ConfigUtilities;
import org.firstinspires.ftc.teamcode.Vision.QualVisionProcessor;
import org.firstinspires.ftc.teamcode.Vision.TrackedSample;
import org.firstinspires.ftc.teamcode.autonomous.AutoConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
@Config
public class RI3WHardware {

    public static final double TICKS_PER_MOTOR_REV = 537.7;
    public static final double DRIVE_GEAR_RATIO = 1.0/1.0;
    public static final double TICKS_PER_DRIVE_REV = TICKS_PER_MOTOR_REV * DRIVE_GEAR_RATIO;
    public static final double WHEEL_DIAMETER = 3.78;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double TICKS_PER_INCH = TICKS_PER_DRIVE_REV / WHEEL_CIRCUMFERENCE;
    public static double TICKS_PER_MOTOR_REV_SLIDES = 1425.1;
    public static double DRIVE_GEAR_RATIO_SLIDES = 1.0/1.0;
    public static double TICKS_PER_DRIVE_REV_SLIDES = TICKS_PER_MOTOR_REV_SLIDES * DRIVE_GEAR_RATIO_SLIDES;
    public static double WHEEL_DIAMETER_SLIDES = 1.42;
    public static double WHEEL_CIRCUMFERENCE_SLIDES = Math.PI * WHEEL_DIAMETER_SLIDES;
    public static double TICKS_PER_INCH_SLIDES = TICKS_PER_DRIVE_REV_SLIDES/WHEEL_CIRCUMFERENCE_SLIDES;
    public static double linearSlidesPower = .1; // Constant speed the linear slides will move at.
    public static double clawOffset = -5.5; // offset when claw is up, in inches
    public static double CLAW_OPEN_POSITION = 0.4;
    public static double CLAW_CLOSED_POSITION = 0.2;
    public final double SLIDESH_MAX = 0.46;
    public final double SLIDESH_MIN = 0.2;

    public void closeClaw() {
        claw.setPosition(CLAW_CLOSED_POSITION);
    }
    public void openClaw() {
        claw.setPosition(CLAW_OPEN_POSITION);
    }

    public DcMotorEx slidesV;
    public Servo slidesH;
    public static int linearSlidesBase = 0;
    public static int highBasket = (int) (44*TICKS_PER_INCH_SLIDES);
    public static int lowBasket = (int) (27*TICKS_PER_INCH_SLIDES);
    public static int lowChamber = (int) (14*TICKS_PER_INCH_SLIDES);
    public static int highChamber = (int) (27*TICKS_PER_INCH_SLIDES);
    public Servo claw;
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public OpenCvCamera camera;
    public VisionPortal visionPortal;
    QualVisionProcessor testProcessor;
    VisionPortal portal;
    public AprilTagProcessor aprilTag;
    public double turnTolerance = 0.5;
    RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
    public IMU imu;
    public static double speedMod = 1;
    public static double Ki = 1;
    // Percent error allowed when approaching target
    public static double tolerance = 0.10;
    public ElapsedTime timer = new ElapsedTime();
    public double errorSum = 0;
    public double power = 0;
    public double error = 0;

    public Telemetry telemetry;
    public TouchSensor touchRight;  // Touch sensor Object
    public TouchSensor touchLeft;

    public double getAngle(){
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return AngleUnit.normalizeDegrees(angles.getYaw(AngleUnit.DEGREES));
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        init(hardwareMap, telemetry, false);
    }

    public void init(HardwareMap hardwareMap, Telemetry _telemetry, boolean useCamera){
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        telemetry = _telemetry;


        String configurationName = ConfigUtilities.getRobotConfigurationName();
        if (configurationName.equals("coachbot")) {
            // Robot configuration: coachbot
            //    0-frontRight  (GoBILDA 5202/3/4 series)  (reverse)
            //    1-backRight   (GoBILDA 5202/3/4 series)  (reverse)
            //    2-frontLeft   (GoBILDA 5202/3/4 series)
            //    3-backLeft    (GoBILDA 5202/3/4 series)
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        try{
            slidesV = hardwareMap.get(DcMotorEx.class, "slidesV");
        }catch(IllegalArgumentException e) {
            slidesV = new MockDcMotor();
            telemetry.addLine("Can't find slide V: making a mock");
        }
        try{
            slidesH = hardwareMap.get(Servo.class, "slidesH");
        }catch(IllegalArgumentException e) {
            slidesH = new MockServo();
            telemetry.addLine("Can't find slide H: making a mock");
        }

        try{
            claw = hardwareMap.get(Servo.class, "claw");
        }catch(IllegalArgumentException e){
            claw = new MockServo();
            telemetry.addLine("Can't find claw: making a mock");
        }
        slidesV.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesV.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesV.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesV.setTargetPosition(linearSlidesBase);
        slidesV.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        try{
            touchLeft = hardwareMap.get(TouchSensor.class, "touchLeft");
        }catch(IllegalArgumentException e){
            touchLeft = new MockTouchSensor();
            telemetry.addLine("Can't find touchLeft: making a mock");
        }

        try{
            touchRight = hardwareMap.get(TouchSensor.class, "touchRight");
        }catch(IllegalArgumentException e){
            touchRight = new MockTouchSensor();
            telemetry.addLine("Can't find touchRight: making a mock");
        }

        logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        usbFacingDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        // Our Control Hub has the new IMU chip (BHI260AP). Use the new generic IMU class when
        // requesting a reference to the IMU hardware. What chip you have can be determined by
        // using "program and manage" tab on dr iver station, then "manage" on the hamburger menu.
        imu = hardwareMap.get(IMU.class, "imu");

        // Use the new RevHubOrientationOnRobot classes to describe how the control hub is mounted on the robot.
        // For the coach bot its mounted Backward / usb cable on the right (as seen from back of robot)
        // Doc: https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Universal-IMU-Interface
        if (configurationName.equals("coachbot")) {
            // Use the new RevHubOrientationOnRobot classes to describe how the control hub is mounted on the robot.
            // For the coach bot its mounted Backward / usb cable on the right (as seen from back of robot)
            // Doc: https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Universal-IMU-Interface
            logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
            usbFacingDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        }

        IMU.Parameters parameters;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbFacingDirection);
        parameters = new IMU.Parameters(orientationOnRobot);
        boolean success = imu.initialize(parameters);

        if(success){
            telemetry.addLine("IMU initialized");
            telemetry.update();
        }

        if(useCamera){
            testProcessor = new QualVisionProcessor(telemetry);
            portal = new VisionPortal.Builder()
                    .addProcessor(testProcessor)
                    .setCameraResolution(new Size(320, 240))
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .build();

            // Hook up the camera to FTC Dashboard
            FtcDashboard.getInstance().startCameraStream(testProcessor, 0);
        }
    }

    public Double getTurnToAngleSpeed(Double turnAngle) {

        if (turnAngle == null) {
            return null;
        }

        //robot.getAngle is between -180 and 180, starting at 0
        double turnPower = 0;
        double targetAngle = AngleUnit.normalizeDegrees(turnAngle) + 180;
        double startAngle = getAngle() + 180;
        double turnError = AngleUnit.normalizeDegrees(targetAngle - startAngle);
            if (turnError >= 0) {
                turnPower = turnError / 90;
                if (turnPower > AutoConfig.getInstance().speed) {
                    turnPower = AutoConfig.getInstance().speed;
                }
            } else if (turnError < 0) {
                turnPower = turnError / 90;
                if (turnPower < -AutoConfig.getInstance().speed) {
                    turnPower = -AutoConfig.getInstance().speed;
                }
        }

            return turnPower;
    }

    public TrackedSample getDetectedSample() {
        return testProcessor.detectedSample;
    }

    public double getVisionPortalWidth() {
        return testProcessor.targetSize.width;
    }

    public void setTeamColor(QualVisionProcessor.SampleColor color) {
        QualVisionProcessor.interestColor = color;

    }

    public QualVisionProcessor.SampleColor getTeamColor() {
        return QualVisionProcessor.interestColor;
    }

    public double getPos(){
        power = 0;
        error = 0;
        TrackedSample detectedSample = getDetectedSample();

        // Tests if there is a sample present
        if(detectedSample != null){
            Point centroid = detectedSample.sample.centroid;
            // Converts centroid x position from pixels to screen percentage
            double cx = centroid.x / getVisionPortalWidth();
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

        timer.reset();
        return power;
    }
}
