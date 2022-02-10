// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import frc.team6429.util.Utils;
import frc.team6429.robot.Constants;

/** Add your docs here. */
public class Dumper {

    private static Dumper mInstance = new Dumper();

    public static Dumper getInstance(){
        return mInstance;
    }

    public WPI_VictorSPX dumperMotor;

    public Indexer mIndexer;
    public Timer mTimer;

    public Dumper(){
        //dumperMotor = new WPI_VictorSPX(Constants.dumperMotorID);
        dumperMotor = Utils.makeVictorSPX(Constants.dumperMotorID, false);

        mIndexer = Indexer.getInstance();
        mTimer = new Timer();
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
    public void dumperSendWithIndexer(double dumperSpeed){
        dumperMotor.set(dumperSpeed);
        mIndexer.indexerOn(1, 1);
    }

    /**
     * Sets Dumper Motor and Indexer to On to Send Cargo Ball/Balls: Seperator wheel, Dumper Rollers and Indexer On
     * @param dumperSpeed
     */
    public void dumperOppositeWithIndexer(double dumperSpeed){
        dumperMotor.set(-dumperSpeed);
        mIndexer.indexerReverse(1, 1);
    }

    /**
     * Stops dumper motor with indexer
     */
    public void dumperStopWithIndexer(){
        dumperMotor.stopMotor();
        mIndexer.indexerStop();
    }

    public void dumperWithTimer(double wantedTime){
        
    }
}
