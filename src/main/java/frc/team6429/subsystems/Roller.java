// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.team6429.robot.Constants;

/** Add your docs here. */
public class Roller {

    private static Roller mInstance = new Roller();

    public static Roller getInstance(){
        return mInstance;
    }

    public WPI_VictorSPX rollerMotor;
    public WPI_VictorSPX seperatorMotor;

    public Roller(){
        rollerMotor = new WPI_VictorSPX(Constants.rollerMotorID);
        seperatorMotor = new WPI_VictorSPX(Constants.seperatorMotorID);
        
    }
}
