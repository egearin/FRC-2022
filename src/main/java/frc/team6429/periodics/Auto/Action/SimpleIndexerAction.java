// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import frc.team6429.subsystems.Indexer;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SimpleIndexerAction implements Action {

    public Indexer mIndexer;
    public Timer timer;
    public double wanted_time;
    public boolean finished;

    public SimpleIndexerAction(double wantedTime){
        mIndexer = Indexer.getInstance();
        timer = new Timer();
        wanted_time = wantedTime;

    }
    @Override
    public void start(){
        timer.reset();
        timer.start();
    }

    @Override
    public void update(){
        mIndexer.indexerOn(1, 0.3);
    }

    @Override
    public boolean isFinished(){
        finished = timer.get() >= wanted_time;
        
        return finished;
    }

    @Override
    public void done(){
        mIndexer.indexerStop();  
    }
    
}
