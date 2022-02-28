// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import edu.wpi.first.wpilibj.PS4Controller;
import frc.team6429.robot.Constants;

/** Add your docs here. */
public class Gampead2 {

    private static Gampead2 mInstance = new Gampead2();

    public static Gampead2 getInstance(){
        return mInstance;
    }

    public PS4Controller gamepad;

    public Gampead2(){
        gamepad = new PS4Controller(Constants.gamepad2Joystick);
    }

    
}
