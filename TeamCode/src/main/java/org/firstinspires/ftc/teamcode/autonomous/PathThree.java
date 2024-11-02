package org.firstinspires.ftc.teamcode.autonomous;

public class PathThree extends RI3WAbstractAutonomous {
    // This program only makes the robot go to park. Needed if robot is not working AT ALL.
    @Override
    public void runOpMode() throws InterruptedException {
        if (Autoconfig.whereToStart == StartingPosition.LEFT){
            //
        } else if (Autoconfig.whereToStart == StartingPosition.RIGHT){
            //
        }

        if (Autoconfig.whereToPark == ParkPosition.CORNER){
            //
        } else if (Autoconfig.whereToPark == ParkPosition.EDGE){
            //
        }
    }
}
