package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "PushBot")
public class PushBot extends RI3WAbstractAutonomous {
// Emergency Auto + Push Bot (pushes 2 yellow samples into Net Zone, Park either position)
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize FTC Dashboard to show the telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);
        setUp();
        waitForStart();
        //waitForDelay();
        if (AutoConfig.getInstance().whereToStart == StartingPosition.LEFT){
            parsePath(
               "Strafe left 20 inches\n" +
               "Move forward 46 inches\n"
            );
        } else if (AutoConfig.getInstance().whereToStart == StartingPosition.RIGHT){
            parsePath(
                "Move forward 23 inches\n" +
                "Strafe left 51 inches\n" +
                "Move forward 23 inches\n"
            );
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

        if (AutoConfig.getInstance().whereToPark == ParkPosition.CORNER){
            parsePath(
                "Strafe right 115 inches\n" +
                "Move backward 22 inches\n"
            );
        } else if (AutoConfig.getInstance().whereToPark == ParkPosition.EDGE){
            parsePath(
                "Strafe right 92 inches\n" +
                "Move backward 22 inches\n"
            );
        }
    }
}
