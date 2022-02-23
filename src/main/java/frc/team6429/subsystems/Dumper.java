// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Timer;

import frc.team6429.util.Utils;
import frc.team6429.robot.Constants;
import frc.team6429.robot.RobotData.DumperMode;
import frc.team6429.robot.RobotData.InSync;
import frc.team6429.robot.RobotData.OpponentBall;

/** Add your docs here. */
public class Dumper {


    private static Dumper mInstance = new Dumper();

    public static Dumper getInstance(){
        return mInstance;
    }

    public WPI_VictorSPX dumperMotor;

    public Indexer mIndexer;
    public Timer timer;

    public Dumper(){
        //dumperMotor = new WPI_VictorSPX(Constants.dumperMotorID);
        dumperMotor = Utils.makeVictorSPX(Constants.dumperMotorID, false);

        mIndexer = Indexer.getInstance();
        timer = new Timer();
    }

    public void setDumperMode(DumperMode dumperMode, double dumperSpeed){
        switch(dumperMode) {
            case DEFAULT:
                dumperSend(dumperSpeed);
                break;
            case OPPOSITE:
                dumperSendOpposite(dumperSpeed);
                break;
            case OFF:
                dumperStop();
                break;
       }
    }

    public void setSyncMode(InSync mode, double dumperSpeed, double intakeSpeed, double conveyorSpeed){
        switch(mode){
            case INSYNC:
                dumperSendWithIndexer(dumperSpeed, intakeSpeed, conveyorSpeed);
                break;
            case OPPINSYNC:
                dumperOppositeWithIndexer(dumperSpeed, intakeSpeed, conveyorSpeed);
                break;
            case OFF:
                dumperStop();
                mIndexer.indexerStop();
        }
    }

    public void opBall(OpponentBall mode, double dumperSpeed, double intakeSpeed, double conveyorSpeed){
        switch(mode){
            case DEFAULT:

                break;    
            case OPPOSITE:
                
                break;
        }
    }

    /**
     * Sets only Dumper Motor to On to Send Cargo Ball/Balls: Seperator wheel and Dumper Rollers On
     * @param speed
     */
    public void dumperSend(double speed){
        dumperMotor.set(speed);
    }

    /**
     * Sets Dumper Motor to Reverse to Send Unneeded Cargo Ball/Balls: Seperator wheel and Dumper Rollers Reverse
     * @param speed
     */
    public void dumperSendOpposite(double speed){
        dumperMotor.set(-speed);
    }

    /**
     * Only stops dumper motor
     */
    public void dumperStop(){
        dumperMotor.stopMotor();
    }

    /**
     * Sets Dumper Motor and Indexer to On to Send Cargo Ball/Balls: Seperator wheel, Dumper Rollers and Indexer On
     * @param dumperSpeed
     */
    public void dumperSendWithIndexer(double dumperSpeed, double intakeSpeed, double conveyorSpeed){
        dumperMotor.set(dumperSpeed);
        mIndexer.indexerOn(intakeSpeed, conveyorSpeed);
    }

    /**
     * Sets Dumper Motor and Indexer to On to Send Cargo Ball/Balls: Seperator wheel, Dumper Rollers and Indexer On
     * @param dumperSpeed
     */
    public void dumperOppositeWithIndexer(double dumperSpeed, double intakeSpeed, double conveyorSpeed){
        dumperMotor.set(-dumperSpeed);
        mIndexer.indexerReverse(intakeSpeed, conveyorSpeed);
    }

    /**
     * Stops dumper motor with indexer
     */
    public void dumperStopWithIndexer(){
        dumperMotor.stopMotor();
        mIndexer.indexerStop();
    }

    public void dumperWithTimer(DumperMode dumperMode, double wantedTime, double dumperSpeed){
        if(timer.get() < wantedTime){
            setDumperMode(dumperMode, dumperSpeed);
        }
        else{
            setDumperMode(DumperMode.OFF, 0);
        }
    }

    public void inSyncWithTimer(InSync inSync, double dumperSpeed, double intakeSpeed, double conveyorSpeed, double wantedTime){
        if(timer.get() < wantedTime){
            setSyncMode(inSync, dumperSpeed, intakeSpeed, conveyorSpeed);
        }
        else{
            setSyncMode(InSync.OFF, 0, 0, 0);
        }
    }
}
