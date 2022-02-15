// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.trajectory.Trajectory;

/** Add your docs here. */
public class RobotData {

    public enum RobotStates{
        STORAGEFULL,
        STORAGEHALVED,
        CLEARINGSTORAGE,
        FILLINGSTORAGE,
        HANGPERIOD,
        WARNING,
        DISABLE;
    }

    public enum CANcoderMode{
    }
    
    public enum LoadedTrajectory{
        NONE,
        DEFAULT,
        TWOCARGO,
        THREECARGO,
        FOURCARGO;

    }

    public static int selectedTrajectory = -1;

    public static List<List<Trajectory>> listOfTrajectories = new ArrayList<>();
    public static Field2d fieldSim = new Field2d();

}