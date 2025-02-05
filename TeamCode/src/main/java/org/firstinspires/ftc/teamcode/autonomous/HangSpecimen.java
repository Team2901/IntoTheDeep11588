package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "HangSpecimen")
public class HangSpecimen extends RI3WAbstractAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        // TODO: Figure out how to make this code compatible with new robot design
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
                    "Claw close\n"+
                            "Lift base\n"+
                            "Move forward 25 inches\n"+
                            "Lift highChamber\n"+
                            "Slide extend 0.5\n"+
                            "Move forward 1.5 inches\n"+
                            "Lower 580\n"+
                            "Slide retract 0.2\n"+
                            "Move backward 3 inches\n"+
                            "Claw open\n"+
                            "Lift base\n"+
                            "Strafe right 31 inches\n"+
                            "Move forward 22 inches\n"+
                            "Strafe right 11 inches\n"+
                            "Move backward 44 inches\n"+
                            "Move forward 16 inches\n"+
                            "TurnA clockwise 180 degrees\n"+
                            "Wait 1000 ms\n" +
                            "Move forward 14 inches\n"+
                            "Lift ground\n"+
                            "Claw close\n" +
                            "Lift base\n"+
                            "Strafe right 50 inches\n"+
                            "TurnA clockwise 0 degrees\n"+
                            "Move forward 22 inches\n"+
                            "Lift highChamber\n"+
                            "Slide extend 0.5\n"+
                            "Move forward 1.5 inches\n"+
                            "Lower 580\n"+
                            "Slide retract 0.2\n"+
                            "Move backward 3 inches\n"+
                            "Claw open\n"+
                            "Lift base\n"

            );
        }

        // Claw code goes after starting positions

//        if (AutoConfig.getInstance().whereToPark == ParkPosition.CORNER){
//            parsePath(
//                    "Strafe right 64 inches\n" +
//                    "Move backward 21 inches\n"
//            );
//        } else if (AutoConfig.getInstance().whereToPark == ParkPosition.EDGE){
//            parsePath(
//                    "Strafe right 58 inches\n" +
//                    "Move backward 21 inches\n"
//            );
//        }
        while(opModeIsActive()){
            idle();
        }
    }

}
