package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.RI3WHardware;

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
                    "Strafe right 24 inches\n" +
                    "Move forward 15 inches\n" +
                    "Strafe right 20 inches\n" +
                    "Move backward 56 inches\n" +
                    "Move forward 15 inches\n"
            );
        } else if (AutoConfig.getInstance().whereToStart == StartingPosition.RIGHT){
            parsePath(
                    //"Lift base\n" +
                    "Strafe right 24 inches\n" +
                    "Move forward 15 inches\n" +
                    "Strafe right 20 inches\n" +
                    "Move backward 56 inches\n"
            );


        }

        //parsePath();

        /*if (AutoConfig.getInstance().whereToPark == ParkPosition.CORNER){
            parsePath();
        } else if (AutoConfig.getInstance().whereToPark == ParkPosition.EDGE){
            parsePath();
        }

         */
    }
}
