package org.firstinspires.ftc.teamcode.autonomous;

public class PathThree extends RI3WAbstractAutonomous {
    // This program only makes the robot go to park. Needed if robot is not working AT ALL.
    @Override
    public void runOpMode() throws InterruptedException {
        if (AutoConfig.getInstance().whereToStart == StartingPosition.LEFT){
            //
        } else if (AutoConfig.getInstance().whereToStart == StartingPosition.RIGHT){
            //
        }

        if (AutoConfig.getInstance().whereToPark == ParkPosition.CORNER){
            //
        } else if (AutoConfig.getInstance().whereToPark == ParkPosition.EDGE){
            //
        }
    }
}
