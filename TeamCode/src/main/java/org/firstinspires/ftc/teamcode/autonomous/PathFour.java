package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.CountDownTimer;

@Autonomous (name = "PathFour")
public class PathFour extends RI3WAbstractAutonomous {
// Emergency Auto + Push Bot (pushes 1 yellow sample into Net Zone, Park either position)
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize FTC Dashboard to show the telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);
        setUp();
        //moveSlides(SlidePosition.base);
//        robot.linearSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.linearSlides.setTargetPosition(1000);
//        robot.linearSlides.setPower(0.25);
        waitForStart();
        //waitForDelay();

        if (whereToStart == StartingPosition.LEFT){
            move(0,-20);
            move(46, 0);
        } else if (whereToStart == StartingPosition.RIGHT){
            move(23, 0);
            move(0, -51);
            move(23, 0);
        }

        parsePath(
             //12 in originally
            "Strafe left 21 inches\n" +
            "Move backward 44 inches\n" +
            "Move forward 46 inches\n" +
            "Strafe left 7 inches\n" +
            "Move backward 18 inches\n" +
            "Strafe left 3 inches\n" +
            "Move forward 20 inches\n" +
            "TurnA 0\n"
        );

        if (whereToPark == ParkPosition.CORNER){
            move(0,115);
            // TODO: some of this moves too far
            move(-22,0);
        } else if (whereToPark == ParkPosition.EDGE){
            move(0,92);
            move(-22,0);
        }
    }
}
