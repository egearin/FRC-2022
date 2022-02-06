// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team6429.robot.Constants;
import frc.team6429.util.Utils;

/** Add your docs here. */
public class Intake {

    private static Intake mInstance = new Intake();

    public static Intake getInstance(){
        return mInstance;
    }

    public WPI_VictorSPX intakeMotor;
    public WPI_VictorSPX conveyorMotor;

    public Intake(){
        //intakeMotor = new WPI_VictorSPX(Constants.intakeMotorID);
        intakeMotor = Utils.makeVictorSPX(Constants.intakeMotorID, false);
        //conveyorMotor = new WPI_VictorSPX(Constants.conveyorMotorID);
        conveyorMotor = Utils.makeVictorSPX(Constants.conveyorMotorID, false);
    }


}
