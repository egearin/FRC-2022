// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import frc.team6429.subsystems.Drive2;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.team6429.robot.Constants;

/** Add your docs here. */
public class Hang {

    private static Hang mInstance = new Hang();

    public static Hang getInstance(){
        return mInstance;
    }

    public Drive mDrive;
    public Drive2 mDrive2;

    public VictorSP hangMotor1;
    public WPI_TalonFX hangMotor2;

    public Encoder traversalEnc;
    public CANCoder hangCANcoder;
    
    public Hang(){
        hangMotor1 = new VictorSP(Constants.hangMotorPort);
        //hangMotor2 = makeTalonFX(Constants.hangMotorID, false);
        hangCANcoder = new CANCoder(Constants.hangCANcoderID);
        mDrive = new Drive();
        mDrive2 = new Drive2();

    }

    private WPI_TalonFX makeTalonFX(int id, boolean invert) { 
        WPI_TalonFX talon = new WPI_TalonFX(id);
    
        talon.configFactoryDefault();
        talon.setInverted(invert);
        talon.stopMotor();
    
        return talon;
      }


}
