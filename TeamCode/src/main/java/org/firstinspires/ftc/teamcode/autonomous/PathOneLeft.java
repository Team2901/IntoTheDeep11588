package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PathOneLeft")
public class PathOneLeft extends RI3WAbstractAutonomous {
    String path =
        "Strafe right 3 inches\n" +
        "Move forward 30 inches clawOffset\n" +
        "Strafe right 64 inches\n" +
        "Move back 24 inches\n";

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        waitForStart();

        try {
            parsePath(path);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        /*
        //Robot strafes 3 in. right
        move(0,3);

        // move forward 30" - clawOffset
        move(30 - robot.clawOffset, 0);

        // Claw code

        // strafe right 64 in.
        move(0,64);

        // move back 24 in.
        move(-24,0);
         */
    }
}
