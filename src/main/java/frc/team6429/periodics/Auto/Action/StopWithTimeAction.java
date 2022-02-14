// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.team6429.subsystems.Drive;

/** Stops With Timer */
public class StopWithTimeAction implements Action{

    public Drive mDrive;
    public Timer timer;
    public double wanted_time;

    public StopWithTimeAction(double wantedTime){
        mDrive = Drive.getInstance();
        timer = new Timer();
        wanted_time = wantedTime;
    }
    @Override
    public void start() {
        timer.reset();
        timer.start();
    }

    @Override
    public void update() {
        mDrive.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= wanted_time;
    }

    @Override
    public void done() {
        
    }
}
