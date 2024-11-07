package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Emergency")
public class Emergency extends RI3WAbstractAutonomous {
    // This program only makes the robot go to park. Needed if robot is not working AT ALL.
    @Override
    public void runOpMode() throws InterruptedException {
        if (AutoConfig.getInstance().whereToStart == StartingPosition.LEFT){
            move(0, 42);
        } else if (AutoConfig.getInstance().whereToStart == StartingPosition.RIGHT){
            move(0, 22);
        }

        if (AutoConfig.getInstance().whereToPark == ParkPosition.CORNER){
            move(0, 28);
        } else if (AutoConfig.getInstance().whereToPark == ParkPosition.EDGE){
            move(0, 5);
        }
    }
}
