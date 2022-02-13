// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.team6429.robot.Constants;

/** Add your docs here. */
public class Drivepanel {

    private static Drivepanel mInstance = new Drivepanel();

    public static Drivepanel getInstance(){
        return mInstance;
    }

    Joystick panel1;
    Joystick panel2;

    public Drivepanel(){
        panel1 = new Joystick(Constants.panel1Joystick);
        panel2 = new Joystick(Constants.panel2Joystick);
    }
    
    //Pivot
    public boolean pivotUp(){
        return panel1.getRawButton(Constants.pivotUpButtonDrivepanel);
    }

    public boolean pivotDown(){
        return panel1.getRawButton(Constants.pivotDownButtonDrivepanel);
    }

    //Indexer
    public boolean getIndexerDrivepanel(){
        return panel1.getRawButton(Constants.indexerOnButtonDrivepanel);
    }
    
    public boolean getIndexerReverseDrivepanel(){
        return panel1.getRawButton(Constants.indexerReverseButtonDrivepanel);
    }

    //Intake
    public boolean getIntakeDrivepanel(){
        return panel1.getRawButton(Constants.intakeOnButtonDrivepanel);
    }

    public boolean getIntakeReverseDrivepanel(){
        return panel1.getRawButton(Constants.intakeReverseButtonDrivepanel);
    }

    //Power Take-Off
    public boolean getPTOpressed(){
        return panel1.getRawButtonReleased(Constants.ptoButton);
    }

    //Conveyor
    public boolean getConveyorDrivepanel(){
        return panel1.getRawButton(Constants.conveyorOnButtonDrivepanel);
    }
    
    public boolean getConveyorReverseDrivepanel(){
        return panel1.getRawButton(Constants.conveyorReverseButtonDrivepanel);
    }

    //Dumper
    public boolean getDumperDrivepanel(){
        return panel1.getRawButton(Constants.dumperOnDrivepanel);
    }
    
    public boolean getDumperReverseDrivepanel(){
        return panel1.getRawButton(Constants.dumperOppositeDrivepanel);
    }

    //Hang 
    public boolean getHangMotorOn(){
        return panel1.getRawButton(Constants.setHangDrivepanel);
    }

    public boolean getHangMotorReverse(){
        return panel1.getRawButton(Constants.setHangReverseDrivepanel);
    }

    //Reset
    public boolean getEncoderReset(){
        return panel1.getRawButton(Constants.encoderResetDrivepanel);
    }

    public boolean getPigeonReset(){
        return panel1.getRawButton(Constants.pigeonResetDrivepanel);
    }
}