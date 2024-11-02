package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Autoconfig {
    public static RI3WAbstractAutonomous.ParkPosition whereToPark = RI3WAbstractAutonomous.ParkPosition.CORNER;
    public static RI3WAbstractAutonomous.StartingPosition whereToStart = RI3WAbstractAutonomous.StartingPosition.LEFT;
    public static double speed = .55;
    public static int delayNumSec = 0;
}
