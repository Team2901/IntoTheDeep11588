package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "PathFourLeft")
public class PathFourLeft extends RI3WAbstractAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        // Robot moves to cage, moves to sample, moves sample to area, and goes back to park

        robot.init(hardwareMap, telemetry);
        waitForStart();

        //Robot strafes left 22 in.
        move(0,-22);
        waitForContinue();
        //Robot moves 45 in. forward
        move(45,0);
        waitForContinue();
        //Robot strafes left 17 in.
        move(0,-17);
        waitForContinue();
        //Robot moves backward 42 in. (stopped here)
        move(-44, 0);
        waitForContinue();
        //Robot strafes left 3 in.
        move(0,-3);
        waitForContinue();
        //Robot moves backward 12 in.
        move(-12,0);
        waitForContinue();
        //Robot strafes right 10 in.
        move(0,10);
        waitForContinue();
        //Robot moves 20 in. forward
        move(20,0);
        waitForContinue();
        //Robot strafes right 92 in.
        move(0,92);
        waitForContinue();
        //Robot moves 20 in. backward
        move(-20,0);
    }

}
