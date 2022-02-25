// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Teleop;

import com.ctre.phoenix.CustomParamConfigUtil;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team6429.subsystems.Drive;
import frc.team6429.subsystems.Drivepanel;
import frc.team6429.subsystems.Dumper;
import frc.team6429.subsystems.Gamepad;
import frc.team6429.subsystems.Climb;
import frc.team6429.subsystems.Indexer;
import frc.team6429.subsystems.LED;
import frc.team6429.util.Sensors;
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
    public LED mLed;
    public Indexer mIndexer;
    public Dumper mDumper;
    public Climb mHang;
    public Gamepad mGamepad;
    public Drivepanel mDrivepanel;
    public Sensors mSensors;
    


    private TeleopPeriodic(){
        mDrive = Drive.getInstance();
        mIndexer = Indexer.getInstance();
        mDumper = Dumper.getInstance();
        mHang = Climb.getInstance();
        mSensors = Sensors.getInstance();
        mGamepad = Gamepad.getInstance();
        mDrivepanel = Drivepanel.getInstance();
        
    }

    public void teleopPeriodic(){
        
        //Custom Indexer Codes
        if(mGamepad.getCustomIndexerOn()){
            mIndexer.intakeOn(1);
        }

        else if (mGamepad.getCustomIndexerReverse()){
            mIndexer.intakeReverse(1);
        }

        else if (mGamepad.getDumperGamepad()){
            mDumper.dumperSendWithIndexer(1, 1, 1);
        }

        else if (mGamepad.getDumperOppositeGamepad()){
            mDumper.dumperOppositeWithIndexer(1, 1, 1);
        }

        else{
            mDumper.dumperStopWithIndexer();
        }

        if(mGamepad.getTestPTO()){
            mDrive.powerTakeOff(!mDrive.pto.get());
        }

    }
}

