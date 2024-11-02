package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.CountDownTimer;

@Autonomous (name = "PathFour")
public class PathFour extends RI3WAbstractAutonomous {
// Emergency Auto + Push Bot (pushes 2 yellow samples into Net Zone, Park either position)
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize FTC Dashboard to show the telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);
        setUp();
        waitForStart();
        //waitForDelay();

        if (Autoconfig.whereToStart == StartingPosition.LEFT){
            move(0,-20);
            move(46, 0);
        } else if (Autoconfig.whereToStart == StartingPosition.RIGHT){
            move(23, 0);
            move(0, -51);
            move(23, 0);
        }

        parsePath(
            "Strafe left 13 inches\n" +
            "Move backward 44 inches\n" +
            "Strafe left 10 inches\n" +
            "Move forward 10 inches\n" +
            "Strafe right 10 inches\n" +
            "Move forward 35 inches\n" +
            "Strafe left 11 inches\n" +
            "Move backward 46 inches\n" +
            "Move forward 25 inches\n"
            //"TurnA 0\n" +
            //telemetry.addData("Has Turned to angle", robot.getAngle())
        );

        if (Autoconfig.whereToPark == ParkPosition.CORNER){
            move(0,115);
            move(-22,0);
        } else if (Autoconfig.whereToPark == ParkPosition.EDGE){
            move(0,92);
            move(-22,0);
        }
    }
}
