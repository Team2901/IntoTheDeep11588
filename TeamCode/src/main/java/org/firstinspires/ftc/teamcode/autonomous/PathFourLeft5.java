package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;

@Autonomous (name = "PathFourLeft5")
public class PathFourLeft5 extends RI3WAbstractAutonomous {

    enum ParkPosition {
        CORNER,
        EDGE
    }

    enum StartingPosition {
        LEFT,
        RIGHT
    }

    ParkPosition whereToPark = ParkPosition.CORNER;
    StartingPosition whereToStart = StartingPosition.LEFT;
    public void help(){
        telemetry.addLine("Set Up Mode");
        telemetry.addLine();
        telemetry.addData("startLocation", whereToStart);
        telemetry.addLine("LB: Staring on left");
        telemetry.addLine("RB: Staring on right");
        telemetry.addLine();
        telemetry.addData("parkValue", whereToPark);
        telemetry.addLine("X: Park edge");
        telemetry.addLine("B: Park corner");
        telemetry.update();
    }
    public void setUp() {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");

        while (!isStarted()){
            help();
            gamepad.update();

            if (gamepad.b.isPressed()){
                whereToPark = ParkPosition.CORNER;
            }

            if (gamepad.x.isPressed()){
                whereToPark = ParkPosition.EDGE;
            }

            if (gamepad.left_bumper.isInitialPress()){
                whereToStart = StartingPosition.LEFT;
            }

            if (gamepad.right_bumper.isInitialPress()){
                whereToStart = StartingPosition.RIGHT;
            }
            telemetry.update();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize FTC Dashboard to show the telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Robot moves to cage, moves to sample, moves sample to area, and goes back to park


        robot.init(hardwareMap, telemetry);
        setUp();
        waitForStart();

        if (whereToStart == StartingPosition.LEFT){
            move(0,-20);
        } else if (whereToStart == StartingPosition.RIGHT){ // TODO this
            move(23, 0);
            move(0, -51);
        }
        //waitForContinue();
        move(46,0);
        //waitForContinue();
        // 13-14 in
        //Robot strafes left 12 in.
        move(0,-12);
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
        turnToAngle(0);

        waitForContinue();

        if (whereToPark == ParkPosition.CORNER){
            move(0,119);
            move(-20,0);
        } else if (whereToPark == ParkPosition.EDGE){
            move(0,92);
            move(-20,0);
        }
    }

}
