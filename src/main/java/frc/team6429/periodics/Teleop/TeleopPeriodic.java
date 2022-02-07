// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team6429.subsystems.Drive;
import frc.team6429.subsystems.Dumper;
import frc.team6429.subsystems.Indexer;
import frc.team6429.util.Drivepanel;
import frc.team6429.util.Gamepad;

/** 
 * Teleop Period
 * Robot Subsystems During Teleop Mode
*/
public class TeleopPeriodic {

    private static TeleopPeriodic mInstance = new TeleopPeriodic();

    public static TeleopPeriodic getInstance(){
        return mInstance;
    }

    //Subsystems
    public Drive mDrive;
    public Indexer mIndexer;
    public Dumper mDumper;
    public Gamepad mGamepad;
    public Drivepanel mDrivepanel;



    public TeleopPeriodic(){
        mDrive = Drive.getInstance();
        mIndexer = Indexer.getInstance();
        mDumper = Dumper.getInstance();
        mGamepad = Gamepad.getInstance();
        mDrivepanel = Drivepanel.getInstance();
    }

    public void teleopPeriodic(){
    
    //Manual Pivot Codes
    if(mDrivepanel.pivotDown()) {
        mIndexer.pivotDown();
    }
    else if(mDrivepanel.pivotUp()) {
        mIndexer.pivotUp();
    }
    else {
        mIndexer.pivotStall();
    }

    //Manual Indexer Codes
    if(mDrivepanel.getIndexerDrivepanel()) {
        mIndexer.indexerOn(1, 0.2);
    }
    else if(mDrivepanel.getIndexerReverseDrivepanel()) {
        mIndexer.indexerReverse(1, 1);
    }
    else {
        mIndexer.indexerStop();
    }

    //Custom Indexer Codes

    



    //Dumper Codes
    if(mGamepad.getDumperGamepad()) {
        mDumper.dumperSend(1);
    }
    else if(mGamepad.getDumperOppositeGamepad()) {
        mDumper.dumperSendOpposite(1);
    }
    
    //Power Take-Off
    if(mGamepad.getPTOpressed()) {
        mDrive.powerTakeOff(!mDrive.pto.get());
    }


    

    

    }
}


