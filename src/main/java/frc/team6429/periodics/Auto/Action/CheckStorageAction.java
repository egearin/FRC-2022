// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.team6429.subsystems.Drive;
import frc.team6429.subsystems.Dumper;
import frc.team6429.subsystems.Indexer;
import frc.team6429.util.Sensors;

/** Add your docs here. */
public class CheckStorageAction implements Action {
    
    public enum SensorMode{
        ONECARGOMODE,
        TWOCARGOMODE;
    }

    public Sensors mSensors;
    public Indexer mIndexer;
    public Dumper mDumper;
    public Drive mDrive;
    public Timer timer;
    public boolean is_empty;
    public double ballCount;
    public double check_time;
    public SensorMode sensorMode;

    public CheckStorageAction(double neededBallCount){
        mSensors = Sensors.getInstance();
        mDrive = Drive.getInstance();
        mDumper = Dumper.getInstance();
        timer = new Timer();
        ballCount = neededBallCount;
    }

    @Override
    public void start() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        
    }
}
