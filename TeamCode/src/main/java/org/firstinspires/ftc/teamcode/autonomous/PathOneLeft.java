package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PathOneLeft")
public class PathOneLeft extends RI3WAbstractAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        waitForStart();

        //Robot strafes 3 in. right
        move(0,3);

        // move forward 30" - clawOffset
        move(30 - robot.clawOffset, 0);

        // Claw code

        // strafe right 64 in.
        move(0,64);

        // move back 24 in.
        move(-24,0);
    }
}
