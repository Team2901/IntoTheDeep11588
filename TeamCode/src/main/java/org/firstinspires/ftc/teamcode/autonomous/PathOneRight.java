package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PathOneRight")
public class PathOneRight extends RI3WAbstractAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        setUp();
        waitForStart();

        if (whereToStart == StartingPosition.LEFT){
            //Robot strafes 3 in. right
            move(0,3);
            // move forward 30" - clawOffset
            move(30 - robot.clawOffset, 0);
        } else if (whereToStart == StartingPosition.RIGHT){
            // move forward 30" - clawOffset
            move(30 - robot.clawOffset, 0);
        }

        // extends lift to drop height
        // retract lift to snapHeight
        // open claw

        // move back 26" - clawOffset
        move(-26 + robot.clawOffset, 0) ;
        // retract lift to ground height

        // move right 40"
        move(0, 52);

        if (whereToPark == ParkPosition.CORNER){
            //
        } else if (whereToPark == ParkPosition.EDGE){
            //
        }
    }
}
