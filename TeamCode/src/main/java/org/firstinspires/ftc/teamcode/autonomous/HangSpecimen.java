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
                    "Strafe left 29 inches\n" +
                            "Move forward 52 inches\n" +
                            "Strafe left 6 inches\n" +
                            "Move backwards 47 inches\n" +
                            "Move forward 49 inches\n" +
                            "Strafe left 8 inches\n"+
                            "Move backwards 47 inches\n"+
                            "Move forward 50 inches\n"+
                            "Strafe right 12 inches\n"
            );
        } else if (AutoConfig.getInstance().whereToStart == StartingPosition.RIGHT) {
            parsePath(
                            "Claw close\n" +
                            "Move forward 30 inches\n"+
                            "Lift highChamber\n"+
                            "Slide extend 0.5\n"+
                            "Lower 100\n"+
                            "Claw open\n"+
                            "Slide retract 0.28\n"+
                            "Lift base\n"+
                            "Move backward 30 inches\n"+
                            "Strafe right 29 inches\n" +
                            "Move forward 52 inches\n" +
                            "Strafe right 6 inches\n" +
                            "Move backwards 47 inches\n" +
                            "Move forward 49 inches\n" +
                            "Strafe right 8 inches\n" +
                            "Move backwards 47 inches\n"
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
