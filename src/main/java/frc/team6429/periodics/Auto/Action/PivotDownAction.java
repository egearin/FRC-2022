// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.team6429.subsystems.Indexer;

/** Add your docs here. */
public class PivotDownAction implements Action {

    public Indexer mIndexer;
    public Timer timer;
    public double wanted_time;
    
    public PivotDownAction(){
        mIndexer = Indexer.getInstance();
        timer = new Timer();
    }

    @Override
    public void start() {
        mIndexer.pivotDown();
        
    }

    @Override
    public void update() {
        mIndexer.pivotStall();
        
    }

    @Override
    public boolean isFinished() {
        
        return false;
    }

    @Override
    public void done() {  
    }

}
