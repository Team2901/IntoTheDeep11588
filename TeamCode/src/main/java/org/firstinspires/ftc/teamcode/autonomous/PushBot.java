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
        //TODO add raise slides so claw doesnt drag on ground

        //waitForDelay();
        if (AutoConfig.getInstance().whereToStart == StartingPosition.LEFT){
            parsePath(
                    "Strafe left 29 inches\n" +
                            "Move forward 52 inches\n" +
                            "Strafe left 6 inches\n" +
                            "Move backwards 47 inches\n" +
                            "Move forward 49 inches\n" +
                            "Strafe left 8 inches\n"+
                            "Move backwards 45 inches\n"+
                            "Move forward 49 inches\n" +
                            "Strafe left 5 inches\n" +
                            "Move backwards 47 inches\n" +
                            "Move forward 50 inches\n"+
                            "Strafe right 12 inches\n"
            );
        } else if (AutoConfig.getInstance().whereToStart == StartingPosition.RIGHT) {
            parsePath(
                    "Strafe right 29 inches\n" +
                            "Move forward 52 inches\n" +
                            "Strafe right 8 inches\n" +
                            "Move backwards 47 inches\n" +
                            "Move forward 49 inches\n" +
                            "Strafe right 8 inches\n" +
                            "Move backwards 45 inches\n" +
                            "Move forward 49 inches\n" +
                            "Strafe right 4 inches\n" +
                            "Move backwards 51 inches\n"
            );
        }

        /*if (AutoConfig.getInstance().whereToPark == ParkPosition.CORNER){
            parsePath();
        } else if (AutoConfig.getInstance().whereToPark == ParkPosition.EDGE){
            parsePath();
        }

        (ROUGH) MEASUREMENTS FOR PUSHBOT AUTO
        right 24
        forward 57
        right 18
        backwards 47
        forward 57
        right 10
        backwards 47
        forwards 57
        right 7
        backwards 57
         */

    }
}
