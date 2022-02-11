// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.team6429.periodics.Auto.Action;

import edu.wpi.first.wpilibj.Timer;

/**
 * Robot waits (in milliseconds)
 */
public class WaitAction implements Action {

    double _time;
    Timer timer;

    public WaitAction(double time){
        _time = time;
        timer = new Timer();
    }
    @Override
    public void start(){
        timer.reset();
        timer.start();
    }

    @Override
    public void update(){
    }

    @Override
    public boolean isFinished(){
        return timer.get() > _time;
    }

    @Override
    public void done(){
    }
}
