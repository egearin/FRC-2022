// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.team6429.subsystems.Drive;
import frc.team6429.subsystems.Dumper;
import frc.team6429.subsystems.Indexer;
import frc.team6429.robot.RobotData.DumperMode;
import frc.team6429.robot.RobotData.InSync;
import frc.team6429.util.Sensors;
import frc.team6429.util.Utils;

/**  */
public class TrajectoryIndexerAction implements Action{

    public enum IndexerCommand{
        ON,
        REVERSE,
        OFF;
    }

    public enum PivotCommand{
        APEX,
        CASCADE,
        HALT;
    }

    public Indexer mIndexer;
    public Dumper mDumper;
    public Drive mDrive;
    public Sensors mSensors;
    public Timer timer;
    public PivotCommand pivotCommand;
    public IndexerCommand indexerCommand;
    public InSync insync;
    public DumperMode dumperMode;
    public boolean isIndexed;
    public boolean canDump;
    public double pivot_time;
    public double startTime;
    public double dumper_speed;
    public double intake_speed;
    public double conveyor_speed;

    public TrajectoryIndexerAction(double pivotTime, InSync inSyncMode ,double dumperSpeed, double intakeSpeed, double conveyorSpeed){
        mIndexer = Indexer.getInstance();
        mDumper = Dumper.getInstance();
        mDrive = Drive.getInstance();
        timer = new Timer();
        pivot_time = pivotTime;
        dumper_speed = dumperSpeed;
        intake_speed = intakeSpeed;
        conveyor_speed = conveyorSpeed;
        insync = inSyncMode;
    }

    @Override
    public void start() {
        
        
    }

    @Override
    public void update() {
       
        Pose2d currentPos = mDrive.getPose();
        if (startTime == -1){
            startTime = timer.get();
            pivotCommand = PivotCommand.CASCADE;
        }
        if (Utils.isRobotInPosition(currentPos, mDrive.indexerOnCheckpoint, new Translation2d(0.5, 0.5))){
            indexerCommand = IndexerCommand.ON; 
        }
        else if (Utils.isRobotInPosition(currentPos, mDrive.indexerOffCheckpoint.plus(new Translation2d(-0.75, 0)), new Translation2d(0.1, 0.1))){
            if (startTime == -2){
                startTime = timer.get();
                pivotCommand = PivotCommand.APEX;
            }
            indexerCommand = IndexerCommand.OFF;
        }

        if (pivotCommand == PivotCommand.CASCADE){
            if(timer.get() - startTime < pivot_time){
                pivotCommand = PivotCommand.CASCADE;
            }
            else{
                pivotCommand = PivotCommand.HALT;
                startTime = -2;
            }
        }
        else if(pivotCommand == PivotCommand.APEX){
            if(timer.get() - startTime < pivot_time){
                pivotCommand = PivotCommand.APEX;
            }
            else{
                pivotCommand = PivotCommand.HALT;
                isIndexed = true;
            }
        }

        switch(pivotCommand){
            case APEX:
            mIndexer.pivotUp();
            break;
            case CASCADE:
            mIndexer.pivotDown();
            break;
            case HALT: 
            mIndexer.pivotStall();
            break;
        }

        switch(indexerCommand){
            case ON:
            mIndexer.indexerOn(0.75, 0.3);
            break;
            case REVERSE:
            mIndexer.indexerReverse(0.75, 0.5);
            break;
            case OFF:
            mIndexer.indexerStop();
            break;
        }
    }

    @Override
    public boolean isFinished() {

        return isIndexed;
    }

    @Override
    public void done() {
        
        
    }
}
