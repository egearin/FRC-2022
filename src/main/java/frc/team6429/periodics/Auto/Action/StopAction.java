/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team6429.periodics.Auto.Action;

import frc.team6429.subsystems.Drive;

/**
 * Robot Stops Driving
 */
public class StopAction implements Action {

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
