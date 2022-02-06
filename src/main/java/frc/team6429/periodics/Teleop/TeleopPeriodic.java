// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team6429.subsystems.Drive;
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

    public Drive mDrive;
    public Gamepad mGamepad;


    public TeleopPeriodic() {
        mDrive = Drive.getInstance();
        mGamepad = Gamepad.getInstance();
    }

    public void teleopPeriodic() {

    if(mGamepad.getPTOpressed()) {
        mDrive.powerTakeOff(!mDrive.pto.get());
        
    }

    }
}


