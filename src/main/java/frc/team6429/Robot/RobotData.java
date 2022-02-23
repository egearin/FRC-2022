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

    public enum LoadedTrajectory{
        NONE,
        DEFAULT,
        TWOCARGO,
        THREECARGO,
        FOURCARGO,
        FIVECARGO;
    }

    public enum PathType{
        TWOCARGO,
        THREECARGO,                                         
        FOURCARGO,
    }

    public enum DumperCommand{
        DUMP,
        OPPOSITE,
        SLOW;
    }

    public enum IndexerCommand{
        ON,
        REVERSE,
        OFF;
    }

    public enum PivotCommand{
        APEX,
        CASCADE,
        HALT;
    }

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll;
    }
    
    public enum DumperMode{
        DEFAULT,
        OPPOSITE,
        OFF;
    }

    public enum AutoMode{
        DEFAULT,
        ALTERNATE;
    }

    public enum InSync{
        INSYNC,
        OPPINSYNC,
        OFF;
    }

    public enum CANcoderMode{
        INTEGRATEDSENSOR,
        CANCODER;
    }
    
    public enum Stop{
        Drive,
        Indexer,
        Dumper,
        DumperWithIndexer,
        Intake,
        Conveyor,
        ALL;
    }

    public enum NeutralMode{
        BRAKE,
        COAST,
        EEPROMSetting;
    }
    
    public enum OpponentBall{
        DEFAULT,
        OPPOSITE;
    }

    public enum FourCargoPaths{
        _01, _02, _03, _04, _05;
    }

    public enum ThreeCargoPaths{
        _01, _02, _03, _04;
    }

    public enum TwoCargoPaths{
        _01, _02;
    }

    public static enum UltrasonicStates{
        DEFAULT(false),
        BALLDETECTED(true);

    public final boolean states;

    UltrasonicStates(boolean isStates){
        states = isStates;
    }
    
    public boolean isStates(){
        return states;
      }
    }   
    
    public static Trajectory trajectory;
    public static int selectedTrajectory = -1;

    public static List<List<Trajectory>> listOfTrajectories = new ArrayList<>();
    public static List<Trajectory> twoCargo = new ArrayList<>();
    public static List<Trajectory> threeCargo = new ArrayList<>();
    public static List<Trajectory> fourCargo = new ArrayList<>();
    public static Field2d fieldSim = new Field2d();

}