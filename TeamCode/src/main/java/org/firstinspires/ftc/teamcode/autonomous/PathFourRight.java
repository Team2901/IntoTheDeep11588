package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PathFourRight")
public class PathFourRight extends RI3WAbstractAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        // Robot moves to cage, turns left 180 degrees, moves to sample, moves sample to area,
        // and goes back to park


        robot.init(hardwareMap, telemetry);
        setUp();
        waitForStart();

        //move 23 in. forward
        move(23, 0);

        if (whereToStart == StartingPosition.LEFT){
            move(0,-20);
            move(46, 0);
        } else if (whereToStart == StartingPosition.RIGHT){
            move(23, 0);
            move(0, -51);
            move(23, 0);
        }

        //waitForContinue();
        //move 28 in. forward  ..?
        move(28, 0);
        //waitForContinue();
        //180 turn negative?
        turnToAngle(-180);

        //strafe right 14 in.
        move(0, 14);
        //waitForContinue();
        //move forward 42 in.
        move(42, 0);
        //waitForContinue();
        // strafe 112 in. left
        //move(0, -112);

        if (whereToPark == ParkPosition.CORNER){
            move(0,118);
            move(-22,0);
        } else if (whereToPark == ParkPosition.EDGE){
            move(0,92);
            move(-22,0);
        }
    }
}
