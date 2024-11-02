package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PathOne")
public class PathOne extends RI3WAbstractAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        // TODO: Figure out how to make this code compatible with new robot design
        robot.init(hardwareMap, telemetry);
        setUp();
        waitForStart();

        if (Autoconfig.whereToStart == StartingPosition.LEFT){
            //Robot strafes 3 in. right
            move(0,3);
            // move(0,18);
            // move forward 30" - clawOffset
            move(30 - robot.clawOffset, 0);
            //move(20 - robot.clawOffset, 0);
        } else if (Autoconfig.whereToStart == StartingPosition.RIGHT){
            // move forward 30" - clawOffset
            move(30 - robot.clawOffset, 0);
            //move(0, -13);
            //move(20 - robot.clawOffset, 0);
        }

        try {
            parsePath(
                    "Strafe right 3 inches\n" +
                    "Move forward 30 inches clawOffset\n" +
                    "Strafe right 64 inches\n" +
                    "Move back 24 inches\n"
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        if (Autoconfig.whereToPark == ParkPosition.CORNER){
            //
        } else if (Autoconfig.whereToPark == ParkPosition.EDGE){
            // move(0, 30)
        }
    }
}
