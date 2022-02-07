// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.team6429.util.Utils;
import frc.team6429.robot.Constants;

/** Add your docs here. */
public class Dumper {

    private static Dumper mInstance = new Dumper();

    public static Dumper getInstance(){
        return mInstance;
    }

    public WPI_VictorSPX dumperMotor;

    public Dumper(){
        //dumperMotor = new WPI_VictorSPX(Constants.dumperMotorID);
        dumperMotor = Utils.makeVictorSPX(Constants.dumperMotorID, false);
    }

    /**
     * Sets Dumper Motor On to Send Cargo Ball/Balls: Seperator wheel and Dumper Rollers On
     * @param speed
     */
    public void dumperSend(double speed){
        dumperMotor.set(speed);
    }

    /**
     * Sets Dumper Motor Reverse to Send Unneeded Cargo Ball/Balls: Seperator wheel and Dumper Rollers Reverse
     * @param speed
     */
    public void dumperSendOpposite(double speed){
        dumperMotor.set(-speed);
    }
}
