// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import frc.team6429.subsystems.Indexer;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class IndexerAction implements Action {

    public Indexer mIndexer;
    public Timer timer;
    public double intake_speed;
    public double conveyor_speed;
    public boolean finished;

    public IndexerAction(double intakeSpeed, double conveyorSpeed){
        mIndexer = Indexer.getInstance();
        timer = new Timer();
        intake_speed = intakeSpeed;
        conveyor_speed = conveyorSpeed;

    }
    @Override
    public void start(){
        timer.reset();
        timer.start();
    }

    @Override
    public void update(){
        mIndexer.runWithBallCount(1, 0.3);
    }

    @Override
    public boolean isFinished(){
        return mIndexer.runWithBallCount(1, 0.3) == 2;
    }

    @Override
    public void done(){
        mIndexer.indexerStop();  
    }
    
}
