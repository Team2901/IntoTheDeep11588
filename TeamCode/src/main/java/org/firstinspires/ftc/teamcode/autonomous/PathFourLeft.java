package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "PathFourLeft")
public class PathFourLeft extends RI3WAbstractAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize FTC Dashboard to show the telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Robot moves to cage, moves to sample, moves sample to area, and goes back to park


        robot.init(hardwareMap, telemetry);
        waitForStart();

        //Robot strafes left 22 in.
        move(0,-23);
        //waitForContinue();
        //Robot moves 46 in. forward
        move(46,0);
        //waitForContinue();
        //Robot strafes left 15 in.
        move(0,-14);
        //waitForContinue();
        // robot moves backward 20 inches
        move(-26, 0);
        //waitForContinue();
        // robot moves to the left 3 inches - to move the block a little
        move(0, -6);
        //waitForContinue();
        //Robot moves backward 42 in.
        // TODO (stopped here)
        move(-18, 0);
        //waitForContinue();
        //Robot strafes left 3 in.
        move(0,-3);
        //waitForContinue();
        //Robot moves backward 12 in.
        move(20,0);
        //waitForContinue();
        //Robot moves 20 in. backward
        //move(14,0);
        //waitForContinue();
        //Robot strafes right 92 in.
        move(0,102);
        //waitForContinue();
        //Robot moves 20 in. backward
        move(-20,0);
    }

}
