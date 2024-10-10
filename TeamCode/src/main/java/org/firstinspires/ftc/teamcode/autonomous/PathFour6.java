package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "PathFour6")
public class PathFour6 extends RI3WAbstractAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize FTC Dashboard to show the telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);
        setUp();
        waitForStart();

        if (whereToStart == StartingPosition.LEFT){
            move(0,-20);
            move(46, 0);
        } else if (whereToStart == StartingPosition.RIGHT){
            move(23, 0);
            move(0, -51);
            move(23, 0);
        }

        parsePath(
            "Strafe left 12 inches\n" +
            "Move backward 26 inches \n" +
            "Strafe left 7 inches\n" +
            "Move backward 18 inches\n" +
            "Strafe left 3 inches\n" +
            "Move forward 20 inches\n" +
            "TurnA 0\n"
        );

        if (whereToPark == ParkPosition.CORNER){
            move(0,118);
            // TODO: some of this moves too far
            move(-22,0);
        } else if (whereToPark == ParkPosition.EDGE){
            move(0,92);
            move(-22,0);
        }
    }

}
