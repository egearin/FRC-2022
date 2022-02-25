// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Teleop;


import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team6429.subsystems.Drive;
import frc.team6429.subsystems.Gamepad;
import frc.team6429.subsystems.LED;

/** 
 * Teleop: Robot Drive
 * Robot Drivebase During Teleop Mode
*/
public class DriveTeleop {

    private static DriveTeleop mInstance = new DriveTeleop();
    
    public static DriveTeleop getInstance(){
        return mInstance;
    }

    private Gamepad mGamepad;
    private Drive mDrive;
    private LED mLed;

    private double rotation;
    private double speed;
    private double sensetiveSteering = 0.5;
    private double steering = 0.75;
    private double turnPID = 0.07;
    
    private DriveTeleop(){
        mGamepad = Gamepad.getInstance();
        mDrive = Drive.getInstance();
        mLed = LED.getInstance();
    }
    
    public void driveTeleop(){
    //Robot Drive
    speed = mGamepad.getForward() - mGamepad.getReverse();

    if (Math.abs(mGamepad.getSensetiveSteering()) > 0.2) {
      rotation = (mGamepad.getSensetiveSteering()) * (sensetiveSteering);
    }

    else {
      rotation = (mGamepad.getSteering()) * (steering);
    }

    //Drive Shifter
    /*if(mGamepad.getDriveShifterPressed()) {
      mDrive.driveShift(!mDrive.shifter.get());
    } */

    if(mGamepad.getDriveShiftOnePressed()) {
      mDrive.driveShiftOne();
    }

    else if(mGamepad.getDriveShiftTwoPressed()) {
      mDrive.driveShiftTwo();
    }

    mDrive.robotDrive(speed, rotation);
    mGamepad.forceFeedback(speed, rotation);
  }
}

