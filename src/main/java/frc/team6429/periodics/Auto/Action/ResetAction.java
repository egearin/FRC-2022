// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import frc.team6429.util.Sensors;

/** Add your docs here. */
public class ResetAction implements Action {

    public Sensors mSensors;
    public Reset which;

    public ResetAction(Reset whichToReset){
        mSensors = Sensors.getInstance();
        which = whichToReset;
    }

    public enum Reset{
        DriveEncoder,
        HangEncoder,
        Gyro,
        ALL;
    }

    public void reset(Reset which){
        switch(which){
            case DriveEncoder:
            mSensors.resetLeftCANcoder();
            mSensors.resetRightCANcoder();
            break;
            case HangEncoder:
            mSensors.resetHangEnc();
            break;
            case Gyro:
            mSensors.gyroReset();
            break;
            case ALL:
            mSensors.resetCANcoder();
            mSensors.resetHangEnc();
            mSensors.gyroReset();
            break;
        }

    }
    @Override
    public void start(){
        reset(which);
    }

    @Override
    public void update(){
        
    }

    @Override
    public boolean isFinished(){
        
        return false;
    }

    @Override
    public void done(){
        
    }

}
