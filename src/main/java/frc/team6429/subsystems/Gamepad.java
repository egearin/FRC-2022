// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.team6429.robot.Constants;
/** Add your docs here. */

public class Gamepad {

    private static Gamepad mInstance = new Gamepad();
    
    public static Gamepad getInstance(){
        return mInstance;
    }

    private PS4Controller gamepad;

    public Gamepad(){
        gamepad = new PS4Controller(Constants.gamepadJoystick);
    }

    public void forceFeedback(double speed, double rotation){
        double leftRotation;
        double rightRotation;
        if (rotation < 0) {
            leftRotation = 0.5 * (Math.abs(rotation) + speed); 
            rightRotation = 0.5 * (Math.abs(speed));
        }

        else {
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

    public boolean getCustomIndexerOn(){
        return gamepad.getRawButton(Constants.customIndexerOnButtonGamepad);
    }

    public boolean getCustomIndexerOff(){
        return gamepad.getRawButton(Constants.customIndexerOffButtonGamepad);
    }

    public boolean getIntakeGamepad(){
        return gamepad.getRawButton(Constants.intakeOnButtonGamepad);
    }

    public boolean getIntakeReverseGamepad(){
        return gamepad.getRawButton(Constants.intakeReverseButtonGamepad);
    }

    public boolean getConveyorGamepad(){
        return gamepad.getRawButton(Constants.conveyorOnButtonGamepad);
    }

    public boolean getConveyorReverseGamepad(){
        return gamepad.getRawButton(Constants.conveyorReverseButtonGamepad);
    }

    public boolean getDriveShifterPressed(){
        return gamepad.getRawButtonReleased(Constants.shifterButton);
    }

    public boolean getDriveShiftOnePressed(){
        return gamepad.getRawButtonReleased(Constants.shifterOneButton);
    }
    
    public boolean getDriveShiftTwoPressed(){
        return gamepad.getRawButtonReleased(Constants.shifterTwoButton);
    }
    
    public boolean getDumperGamepad(){
        return gamepad.getRawButton(Constants.dumperButtonGamepad);
    }

    public boolean getDumperOppositeGamepad(){
        return gamepad.getRawButton(Constants.dumperOppositeButtonGamepad);
    }
}
