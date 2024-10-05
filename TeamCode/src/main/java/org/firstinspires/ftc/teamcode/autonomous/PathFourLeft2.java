package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;

@Autonomous (name = "PathFourLeft2")
public class PathFourLeft2 extends RI3WAbstractAutonomous {

    enum ParkPosition {
        CORNER,
        EDGE
    }

    ParkPosition whereToPark = ParkPosition.CORNER;
    public void help(){
        telemetry.addLine("Configure Mode");
        telemetry.addData("currentValue", whereToPark);
        telemetry.addLine("X: Park edge");
        telemetry.addLine("B: Park corner");
        telemetry.update();
    }
    public void configuration() {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");

        while (!isStarted()){
            help();
            gamepad.update();

            if (gamepad.b.isPressed()){
                ParkPosition whereToPark = ParkPosition.CORNER;
            }

            if (gamepad.x.isPressed()){
                ParkPosition whereToPark = ParkPosition.EDGE;
            }
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize FTC Dashboard to show the telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Robot moves to cage, moves to sample, moves sample to area, and goes back to park


        robot.init(hardwareMap, telemetry);
        waitForStart();
        // robot strafes left 23 in
        // 20 in
        //waitForContinue();
        move(0,-20);
        move(46,0);
        //waitForContinue();
        // 13-14 in
        //Robot strafes left 14 in.
        move(0,-14);
        move(-26, 0);
        //waitForContinue();
        // robot moves to the left 6 inches - to move the block a little
        // 5 in
        move(0, -7);
        move(-18, 0);
        //waitForContinue();
        //Robot strafes left 3 in.
        move(0,-3);
        move(20,0);



        //waitForContinue();
        //Robot strafes right 102 in.
        //92
        move(0,92);
        move(-25,0);


    }

}
