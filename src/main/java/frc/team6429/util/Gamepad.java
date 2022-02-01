// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.util;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.team6429.robot.Constants;
/** Add your docs here. */

public class Gamepad {

    private static Gamepad mInstance = new Gamepad();
    
    public static Gamepad getInstance(){
        return mInstance;
    }

    public PS4Controller gamepad;

    

    public void forceFeedback(double speed, double rotation){
        double leftRotation;
        double rightRotation;
        if (rotation < 0){
            leftRotation = 0.5 * (Math.abs(rotation) + speed); 
            rightRotation = 0.5 * (Math.abs(speed));
        }

        else{
            leftRotation = 0.5 * Math.abs(speed);
            rightRotation = 0.5 * (Math.abs(rotation) + speed);
        }
        gamepad.setRumble(RumbleType.kLeftRumble, leftRotation);
        gamepad.setRumble(RumbleType.kRightRumble, rightRotation);
    }

    public double getForward(){
        return gamepad.getRawAxis(Constants.axis_forward);
    }

    public double getReverse(){
        return gamepad.getRawAxis(Constants.axis_reverse);
    }

    public double getSteering(){
        return gamepad.getRawAxis(Constants.axis_steering);
    }

    public double getSensetiveSteering(){
        return gamepad.getRawAxis(Constants.axis_sensetiveSteering);
    }

    
}
