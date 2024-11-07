package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "HangSpecimen")
public class HangSpecimen extends RI3WAbstractAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        // TODO: Figure out how to make this code compatible with new robot design
        robot.init(hardwareMap, telemetry);
        setUp();
        waitForStart();

        if (AutoConfig.getInstance().whereToStart == StartingPosition.LEFT){
            parsePath(
                    "Strafe right 5 inches\n" +
                    "Move forward 21 inches\n"
                    //  - robot.clawOffset
                    // TODO: add claw offset to parse path
            );
        } else if (AutoConfig.getInstance().whereToStart == StartingPosition.RIGHT){
            parsePath(
                    "Strafe left 5 inches\n" +
                    "Move forward 21 inches\n"
            );
        }

        // Claw code goes after starting positions

        if (AutoConfig.getInstance().whereToPark == ParkPosition.CORNER){
            parsePath(
                    "Strafe right 64 inches\n" +
                    "Move backward 21 inches\n"
            );
        } else if (AutoConfig.getInstance().whereToPark == ParkPosition.EDGE){
            parsePath(
                    "Strafe right 58 inches\n" +
                    "Move backward 21 inches\n"
            );
        }
    }

    //try {
        //parsePath(
                //"Strafe right 64 inches\n" +
                        //"Move back 24 inches\n"
       // );
    //} catch (Exception e) {
        //throw new RuntimeException(e);
    //}
}
