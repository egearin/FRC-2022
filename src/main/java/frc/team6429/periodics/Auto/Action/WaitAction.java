/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
