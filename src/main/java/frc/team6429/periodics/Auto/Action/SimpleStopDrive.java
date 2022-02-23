// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.team6429.periodics.Auto.Action;

import frc.team6429.subsystems.Drive;

/**
 * Robot Stops Driving
 */
public class SimpleStopDrive implements Action {

    public Drive mDrive;

    @Override
    public void start(){
        mDrive = Drive.getInstance();
        mDrive.stopDrive();
    }

    @Override
    public void update(){
        mDrive.stopDrive();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
    @Override
    public void done(){
    }

 

}
