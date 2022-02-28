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

    private Gamepad(){
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

    public double getSteering(){
        return gamepad.getRawAxis(Constants.axis_steering); //axis 0
    }

    public double getReverse(){
        return gamepad.getRawAxis(Constants.axis_reverse); //axis 2
    }


    public double getForward(){
        return gamepad.getRawAxis(Constants.axis_forward); //axis 3
    }
 
    public double getSensetiveSteering(){
        return gamepad.getRawAxis(Constants.axis_sensetiveSteering); //axis 4
    }

    //---Two Options---\\
    public boolean getCustomIndexerOn(){
        return gamepad.getRawButton(Constants.customIndexerOnButtonGamepad); //button 1
    }

    public boolean getCustomIndexerReverse(){
        return gamepad.getRawButton(Constants.customIndexerOffButtonGamepad); //button 2
    }
    //-----------------\\

    public boolean getDumperGamepad(){
        return gamepad.getRawButton(Constants.dumperButtonGamepad); //button 3
    }

    public boolean getDumperOppositeGamepad(){
        return gamepad.getRawButton(Constants.dumperOppositeButtonGamepad); //button 4
    }

    //High Gear 
    public boolean getDriveShiftTwoPressed(){
        return gamepad.getRawButtonReleased(Constants.shifterTwoButton); //button 9
    }

    //Low Gear 
    public boolean getDriveShiftOnePressed(){
        return gamepad.getRawButtonReleased(Constants.shifterOneButton); //button 10
    }

    public void getPivotDownGamepad(){
    }
    
    public void getPivotUpGamepad(){

    }

    //----Not Used----\\
    public boolean getIntakeGamepad(){
        return gamepad.getRawButton(Constants.intakeOnButtonGamepad); //disabled function
    }

    public boolean getIntakeReverseGamepad(){
        return gamepad.getRawButton(Constants.intakeReverseButtonGamepad); //disabled function
    }

    public boolean getConveyorGamepad(){
        return gamepad.getRawButton(Constants.conveyorOnButtonGamepad); //disabled function
    }

    public boolean getConveyorReverseGamepad(){
        return gamepad.getRawButton(Constants.conveyorReverseButtonGamepad); //disabled function
    }

    public boolean getDriveShifterPressed(){
        return gamepad.getRawButtonReleased(Constants.shifterButton); //disabled function
    }

    public boolean getTestPTO(){
        return gamepad.getRawButtonReleased(6);
    }

    public boolean getTestBrake(){
        return gamepad.getRawButtonReleased(5);
    }
}
