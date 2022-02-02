// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.util;

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
}
