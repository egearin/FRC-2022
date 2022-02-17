// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.team6429.robot.RobotData.IndexerCommand;
import frc.team6429.robot.RobotData.PivotCommand;
import frc.team6429.subsystems.Indexer;
import frc.team6429.util.Sensors;

/** Add your docs here. */
public class AlternateTrajectoryIndexer implements Action{

    public Indexer mIndexer;
    public IndexerCommand indexerCommand;
    public PivotCommand pivotCommand;
    public Sensors mSensors;
    public Timer timer;
    public double prev_time;
    public double ballCount;
    public boolean canDump;
    

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
