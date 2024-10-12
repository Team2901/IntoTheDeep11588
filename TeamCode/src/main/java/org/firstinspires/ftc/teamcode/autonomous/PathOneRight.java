package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PathOneRight")
public class PathOneRight extends RI3WAbstractAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        setUp();
        waitForStart();
        //TODO get robot to stop in same place for both left and right.
        if (whereToStart == StartingPosition.LEFT){
            //Robot strafes 3 in. right
            move(0,18);
            move(20 - robot.clawOffset, 0);
            // move forward 30" - clawOffset
            move(20 - robot.clawOffset, 0);
        } else if (whereToStart == StartingPosition.RIGHT){
            // move forward 30" - clawOffset
            move(0, -13);
            move(20 - robot.clawOffset, 0);
        }

        // extends slides to drop height
        // retract slides to snapHeight
        // open claw

        // move back 26" - clawOffset
        // Put this into ParsePath
        //move(-22 + robot.clawOffset, 0) ;
        // retract lift to ground height

        // move right 40"
        //move(0, 52);
        // TODO park in corner and edge
        if (whereToPark == ParkPosition.CORNER){
            //
        } else if (whereToPark == ParkPosition.EDGE){
            // move(0, 30)
        }
    }
}
