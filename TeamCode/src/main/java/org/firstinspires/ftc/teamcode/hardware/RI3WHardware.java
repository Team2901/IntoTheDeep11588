package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

public class RI3WHardware {

    public static final double TICKS_PER_MOTOR_REV = 537.7;
    public static final double TICKS_PER_MOTOR_REV_ARM = 2786.2;
    public static final double DRIVE_GEAR_RATIO = 1.0/1.0;
    public static final double TICKS_PER_DRIVE_REV = TICKS_PER_MOTOR_REV * DRIVE_GEAR_RATIO;
    public static final double WHEEL_DIAMETER = 3.78;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double TICKS_PER_INCH = TICKS_PER_DRIVE_REV / WHEEL_CIRCUMFERENCE;
    public static double TICKS_PER_MOTOR_REV_SLIDES = 1425.1;
    public static double DRIVE_GEAR_RATIO_SLIDES = 1; //TODO: Unknown for now
    public static double TICKS_PER_DRIVE_REV_SLIDES = TICKS_PER_MOTOR_REV_SLIDES * DRIVE_GEAR_RATIO_SLIDES;
    public static double WHEEL_DIAMETER_SLIDES = 1.42;
    public static double WHEEL_CIRCUMFERENCE_SLIDES = Math.PI * WHEEL_DIAMETER_SLIDES;
    public static double TICKS_PER_INCH_SLIDES = TICKS_PER_DRIVE_REV_SLIDES/WHEEL_CIRCUMFERENCE_SLIDES;
    public static double linearSlidesPower = .25; // Constant speed the linear slides will move at.
    public static double clawOffset = -5.5; // offset when claw is up, in inches
    public static int CLAW_OPEN_POSITION = 0;
    public static int CLAW_CLOSED_POSITION = 0;
    public static double CONT_INTAKE_POWER = .5;

    public void closeClaw() {
        claw.setPosition(CLAW_CLOSED_POSITION);
    }
    public void openClaw() {
        claw.setPosition(CLAW_OPEN_POSITION);
    }
    public void runContIntake() {
        contIntake.setPower(CONT_INTAKE_POWER);
    }
    public void stopContIntake() {
        contIntake.setPower(0);
    }

    public DcMotorEx linearSlides;
    public static int linearSlidesBase = 0;
    public static int highBasket = 0;
    public static int lowBasket = 0;
    public static int lowChamber = 0;
    public static int highChamber = 0;
    public Servo claw;
    public DcMotorEx arm; // this is an arm
    public CRServo contIntake;
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public double speed = .55;
    public OpenCvCamera camera;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    public double turnTolerance = 0.5;
    RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
    public IMU imu;

    public double getAngle(){
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return AngleUnit.normalizeDegrees(angles.getYaw(AngleUnit.DEGREES));
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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
        try {
            arm = hardwareMap.get(DcMotorEx.class, "arm");
        } catch (IllegalArgumentException e){
            arm = new MockDcMotor();
        }
        try {
            linearSlides = hardwareMap.get(DcMotorEx.class, "linearSlides");
            linearSlides.setPower(linearSlidesPower);
            linearSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linearSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlides.setTargetPosition(linearSlidesBase);
        } catch (IllegalArgumentException e){
            linearSlides = new MockDcMotor();
        }
        try {
            claw = hardwareMap.get(Servo.class, "claw");
        } catch (IllegalArgumentException e){
            claw = new MockServo();
        }
        try {
            contIntake = hardwareMap.get(CRServo.class, "contIntake");
        } catch (IllegalArgumentException e){
            contIntake = new MockCRServo();
        }

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        usbFacingDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        // Our Control Hub has the new IMU chip (BHI260AP). Use the new generic IMU class when
        // requesting a reference to the IMU hardware. What chip you have can be determined by
        // using "program and manage" tab on dr iver station, then "manage" on the hamburger menu.
        imu = hardwareMap.get(IMU.class, "imu");

        // Use the new RevHubOrientationOnRobot classes to describe how the control hub is mounted on the robot.
        // For the coach bot its mounted Bgackward / usb cable on the right (as seen from back of robot)
        // Doc: https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Universal-IMU-Interface

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbFacingDirection);
        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        boolean success = imu.initialize(parameters);
        if(success){
            telemetry.addLine("IMU initialized");
            telemetry.update();
        }
    }
}
