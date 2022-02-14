// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.team6429.subsystems.Drive;
import frc.team6429.subsystems.Dumper;
import frc.team6429.subsystems.Indexer;

/** Select subsystem to stop, includes stop all */
public class StopAllAction implements Action{

    public Drive mDrive;
    public Indexer mIndexer;
    public Dumper mDumper;
    public Timer timer;
    public double wanted_time;
    public Stop which;

    public StopAllAction(double wantedTime, Stop whichTStop){
        mDrive = Drive.getInstance();
        mIndexer = Indexer.getInstance();
        mDumper = Dumper.getInstance();
        timer = new Timer();
        wanted_time = wantedTime;
        which = whichTStop;
    }
    
    public enum Stop{
        Drive,
        Indexer,
        Dumper,
        DumperWithIndexer,
        Intake,
        Conveyor,
        ALL;
    }

    public void stop(Stop which){
        switch(which){
            case Drive:
            mDrive.stopDrive();
            break;
            case Indexer:
            mIndexer.indexerStop();
            break;
            case Dumper:
            mDumper.dumperStop();
            break;
            case DumperWithIndexer:
            mDumper.dumperStopWithIndexer();
            break;
            case Intake:
            mIndexer.intakeStop();
            break;
            case Conveyor:
            mIndexer.conveyorStop();
            break;
            case ALL:
            mDrive.stopDrive();
            mIndexer.indexerStop();
            mDumper.dumperStop();
            mDumper.dumperStopWithIndexer();
            mIndexer.intakeStop();
            mIndexer.conveyorStop();
            break;
        }

    }
    @Override
    public void start() {
    
        
    }

    @Override
    public void update() {
        stop(which);
        
    }

    @Override
    public boolean isFinished() {

        return timer.get() >= wanted_time;
    }

    @Override
    public void done() {
      
        
    }

}
