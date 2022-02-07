// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.team6429.subsystems.Dumper;

/** Add your docs here. */
public class SimpleDumperAction implements Action{

    public Dumper mDumper;
    public Timer timer;
    public double wanted_time;

    public SimpleDumperAction(double wantedTime){
        mDumper = Dumper.getInstance();
        timer = new Timer();
        wanted_time = wantedTime;
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
        
    }}
