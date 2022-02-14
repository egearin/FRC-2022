// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.team6429.subsystems.Drive;
import frc.team6429.subsystems.Dumper;
import frc.team6429.util.Sensors;

/** Add your docs here. */
public class TrajectoryDumperAction implements Action {

    public enum DumperCommand{
        DUMP,
        OPPOSITE,
        SLOW;
    }

    public Dumper mDumper;
    public Drive mDrive;
    public Sensors mSensors;
    public Timer timer;
    public boolean dumperWay;
    public double wanted_time;
    public double ballsToGo;
    public DumperCommand dumperCommand;

    //TODO:add parameters
    public TrajectoryDumperAction(){
        mDumper = Dumper.getInstance();
        mDrive = Drive.getInstance();
        mSensors = Sensors.getInstance();
        timer = new Timer();
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
