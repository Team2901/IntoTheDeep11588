package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PathOneRight")
public class PathOneRight extends RI3WAbstractAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        waitForStart();
        // extends lift to drop height

        // move forward 30" - clawOffset
        move(30 - robot.clawOffset, 0);
        // retract lift to snapHeight

        // open claw

        // move back 26" - clawOffset
        move(-26 + robot.clawOffset, 0) ;
        // retract lift to ground height

        // move right 40"
        move(0, 52);
    }
}
