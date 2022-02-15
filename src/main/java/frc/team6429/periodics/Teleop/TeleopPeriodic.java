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
import frc.team6429.subsystems.Hang;
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
    public Hang mHang;
    public Gamepad mGamepad;
    public Drivepanel mDrivepanel;
    public Sensors mSensors;
    


    public TeleopPeriodic(){
        mDrive = Drive.getInstance();
        mIndexer = Indexer.getInstance();
        mDumper = Dumper.getInstance();
        mHang = Hang.getInstance();
        mSensors = Sensors.getInstance();
        mGamepad = Gamepad.getInstance();
        mDrivepanel = Drivepanel.getInstance();
        
    }

    public void teleopPeriodic(){

    //Manual Only Pivot Codes
    if(mDrivepanel.pivotDown()) {
        mIndexer.pivotDown();
    }
    else if(mDrivepanel.pivotUp()) {
        mIndexer.pivotUp();
    }
    else {
        mIndexer.pivotStall();
    }

    //Manual Only Intake and Conveyor Codes
    if(mDrivepanel.getIntakeDrivepanel()) {
        mIndexer.intakeOn(1);
    }
    else if(mDrivepanel.getIntakeReverseDrivepanel()) {
        mIndexer.intakeReverse(1);
    }
    else if(mDrivepanel.getConveyorDrivepanel()) {
        mIndexer.conveyorOn(1);
    }
    else if(mDrivepanel.getConveyorReverseDrivepanel()) {
        mIndexer.conveyorReverse(1);
    }
    else if(mDrivepanel.getIndexerDrivepanel()) {
        mLed.setColorFlow(0, 255, 0, 0, 1, 8, ColorFlowAnimation.Direction.Forward);
        mIndexer.indexerOn(1, 1);
    }
    else if(mDrivepanel.getIndexerReverseDrivepanel()){
        mLed.setColorFlow(0, 255, 0, 0, 1, 8, ColorFlowAnimation.Direction.Backward);
        mIndexer.indexerReverse(1, 1);
    }
    else {
        mIndexer.intakeStop();
        mIndexer.conveyorStop();
    }

    //Custom Indexer
    if(mGamepad.getCustomIndexerOn()) {
        if(mSensors.getBallCount() == 1){
            mIndexer.conveyorStop();
            mIndexer.intakeOn(0.5);
            mLed.setTwinkle(0, 255, 255, 100, 1, 8, TwinklePercent.Percent100);
        }
        else if(mSensors.getBallCount() == 2){
            mIndexer.customIndexerOff();
            mLed.setTwinkle(255, 0, 255, 100, 1, 8, TwinklePercent.Percent100);
        }
        else{
            mIndexer.customIndexerOn();
            mLed.setColorFlow(0, 255, 0, 0, 1, 8, ColorFlowAnimation.Direction.Forward);
        }
    }
    else{
        mIndexer.indexerStop();
    }

     //Dumper Codes
    if(mGamepad.getDumperGamepad()) {
        mLed.setColorFlow(255, 0, 0, 0, 1, 8, ColorFlowAnimation.Direction.Forward);
        mIndexer.indexerOn(1, 1);
        mDumper.dumperSend(1);
    }
    else if(mGamepad.getDumperOppositeGamepad()) {
        mLed.setColorFlow(255, 0, 0, 0, 1, 8, ColorFlowAnimation.Direction.Backward);
        mIndexer.indexerOn(1, 1);
        mDumper.dumperSendOpposite(1);
    }
    else{
        mDumper.dumperStop();
        mIndexer.indexerStop();
    }
    
    //Power Take-Off
    if(mDrivepanel.getPTOpressed()) {
        mLed.setTwinkle(0, 0, 255, 0, 1, 8, TwinklePercent.Percent100);
        mDrive.powerTakeOff(!mDrive.pto.get());
    }

    //Manual Only Dumper Codes
    if(mDrivepanel.getDumperDrivepanel()){
        mLed.setColorFlow(255, 0, 0, 0, 1, 8, ColorFlowAnimation.Direction.Forward);
        mDumper.dumperSend(1);
    }
    else if(mDrivepanel.getDumperReverseDrivepanel()){
        mLed.setColorFlow(255, 0, 0, 0, 1, 7, ColorFlowAnimation.Direction.Backward);
        mDumper.dumperSendOpposite(1);
    }
    else{
        mDumper.dumperStop();
    }

    //Manual Hang Codes 
    if(mDrivepanel.getHangMotorOn()) {
        mLed.setColorFlow(125, 125, 0, 0, 1, 8, ColorFlowAnimation.Direction.Forward);
        mHang.simpleHangMotorUp(0.5);
    }
    else if(mDrivepanel.getHangMotorReverse()) {
        mLed.setColorFlow(125, 125, 0, 0, 1, 8, ColorFlowAnimation.Direction.Backward);
        mHang.simpleHangMotorDown(0.5);
    }
    else{
        mHang.hangStop();
    }

    //Reset Codes
    if(mDrivepanel.getEncoderReset()){
        mLed.setTwinkle(0, 0, 0, 255, 1, 8, TwinklePercent.Percent100);
        mSensors.resetCANcoder();
    }
    else if(mDrivepanel.getPigeonReset()){
        mLed.setTwinkle(0, 0, 0, 255, 1, 8, TwinklePercent.Percent100);
        mSensors.gyroReset();
    }
    else {
    }
    
    }
}


