package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoConfig {
    private static AutoConfig instance;
    public RI3WAbstractAutonomous.ParkPosition whereToPark = RI3WAbstractAutonomous.ParkPosition.CORNER;
    public RI3WAbstractAutonomous.StartingPosition whereToStart = RI3WAbstractAutonomous.StartingPosition.LEFT;
    public double speed = .55;
    public int delayNumSec = 0;
    public static AutoConfig getInstance(){
        if (instance == null){
            instance = new AutoConfig();
        }
        return instance;
    }
    public static void setInstance(AutoConfig newInstance){
        if(instance == null){
            instance = newInstance;
        }else{
            instance.whereToPark = newInstance.whereToPark;
            instance.speed = newInstance.speed;
            instance.delayNumSec = newInstance.delayNumSec;
            instance.whereToStart = newInstance.whereToStart;
        }

    }
}
