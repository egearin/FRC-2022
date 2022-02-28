// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import frc.team6429.util.Utils;

/** 
 * All Robot Constants 
*/
public class Constants {

    //Drive Motor Ports  "TalonFX"
    public static final int driveLeftMotorID = 0; //TalonFX ok
    public static final int driveRightMotorID = 1; //TalonFX ok

    //Other Subsystems Motor Ports and IDs "both"
    public static final int intakeMotorID = 3; //VictorSPX ok
    public static final int dumperMotorID = 2; //TalonFX ok
    public static final int hangMotorID = 4; //TalonFX ok
    public static final int conveyorMotorID = 6; //VictorSPX ok

    //Hub
    public static final int pdhID = 12; //REV Power Distribution Hub ok
    public static final int phID = 5;  //REV Pneumatics Hub ok

    //CANdle Port
    public static final int candleID = 10; //CTR Candle

    //Pigeon port 
    public static final int pigeonID = 7; //Pigeon IMU ok

    //CANcoder Ports
    public static final int leftCANcoderID = 8; //CTR CANcoder ok
    public static final int rightCANcoderID = 9; //CTR CANcoder ok
    
    //public static final int hangCANcoderID = 100;

    //Solenoid Ports
    public static final int shifterChannel = 8; //Single Solenoid for REVPH ok (9)
    public static final int ptoChannel = 13; //Single Solenoid for REVPH ok

    public static final int pivotPistonsForwardChannel = 12; //Double Solenoid for REVPH 
    public static final int pivotPistonsReverseChannel = 10; //Double Solenoid for REVPH

    public static final int midRungLockChannel = 11;
    public static final int frictionBrakeChannel = 9;
    
    //PID Constants
    public static final double kDriveP = 0;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;

    //Feedforward Constants
    public static final double kDriveS = 0;
    public static final double kDriveV = 0;
    public static final double kDriveA = 0;

    public static final double maxVoltageInput = 12;

    //Ultrasonic Ports
    public static final int trigPinHigh = 8;
    public static final int trigPinLow= 9;
    
    public static final int higherSensor = 2;
    public static final int lowerSensor = 3;
    
    //Beam Ports
    public static final int higherBeamChannel = 5;
    public static final int lowerBeamChannel = 6;

    //Drive Constants
    public static final double kTrackWidth = 0;
    public static final DifferentialDriveKinematics kDriveKinematics 
    = new DifferentialDriveKinematics(kTrackWidth);
    public static final double kMaxSpeed = 0;
    public static final double kMaxAcceleration = 0;
    public static final double kRamseteB = 0;
    public static final double kRamseteZeta = 0;
    public static final double kTrajectoryP = 0;

    public static final double shifterPulseDuration = 1;
    public static final double ptoPulseDuration = 1;

    //Gamepad and Drivepanel Constants
    public static final int gamepadJoystick = 0;
    public static final int gamepad2Joystick = 1;
    public static final int panel1Joystick = 1;
    public static final int panel2Joystick = 2;

    //---Gamepad---
    public static final int axis_forward = 3;
    public static final int axis_reverse = 2;
    public static final int axis_steering = 0;
    public static final int axis_sensetiveSteering = 4;

    public static final int shifterButton = 6;
    public static final int shifterOneButton = 10;
    public static final int shifterTwoButton = 9;

    public static final int ptoButton = 6;

    public static final int customIndexerOnButtonGamepad = 1;
    public static final int customIndexerOffButtonGamepad = 2;

    public static final int intakeOnButtonGamepad = 0;
    public static final int intakeReverseButtonGamepad = 0;

    public static final int conveyorOnButtonGamepad = 0;
    public static final int conveyorReverseButtonGamepad = 0;

    public static final int dumperButtonGamepad = 3;
    public static final int dumperOppositeButtonGamepad = 4;

    //---Drivepanel---
    public static final int pivotUpButtonDrivepanel = 2;
    public static final int pivotDownButtonDrivepanel = 3;

    public static final int indexerOnButtonDrivepanel = 4;
    public static final int indexerReverseButtonDrivepanel = 7;

    public static final int intakeOnButtonDrivepanel = 0;
    public static final int intakeReverseButtonDrivepanel = 0;

    public static final int conveyorOnButtonDrivepanel = 0;
    public static final int conveyorReverseButtonDrivepanel = 0;

    public static final int dumperOnDrivepanel = 0;
    public static final int dumperOppositeDrivepanel = 0;

    public static final int setHangDrivepanel = 0;
    public static final int setHangReverseDrivepanel = 0;
    public static final int getRobotHangDrivepanel = 0;

    public static final int encoderResetDrivepanel = 0;
    public static final int pigeonResetDrivepanel = 0;

    
    //Distance Values
    public static final double ultrasonicDistanceAcross = 50.5;
        
    /**
     * to choose subtrahend value to subtract from distance across the conveyor
     * @param value
     * @return
     */
    public static final double subtractedDistance(int value){
        final double subtractedDistanceOne = 5.5;
        final double subtractedDistanceTwo = 10.5;
        final double subtractedDistanceThree = 15.5;

        double[] ret_dist = new double[3];
        ret_dist[0] = subtractedDistanceOne;
        ret_dist[1] = subtractedDistanceTwo;
        ret_dist[2] = subtractedDistanceThree;
    
        return ret_dist[value];
    }

    public static final double degreeCoefficientCANcoder = 7.792562;
    public static final double wheelPerimeter = 0.31918581360472299303;
    public static final double canCoderCPR = 4096;
    public static final double robotMass = 35;
    
    //Necessary Constants
    public static final double traversalAngle = 0;
    public static final double climbPressureFront = 0;
    public static final double pressureTolerance = 0;
    public static final double speedDeadZone = 0;
    public static final double climbTime = 0;
    public static final double climbSpeed = 0;
    
    //Field dimensions
    public static final double fieldLength 
    = Utils.conversion_inchToMeters(54.0 * 12.0);
    public static final double fieldWidth 
    = Utils.conversion_inchToMeters(27.0 * 12.0);
    public static final double hangarLength 
    = Utils.conversion_inchToMeters(128.75);
    public static final double hangarWidth 
    = Utils.conversion_inchToMeters(116.0);

    //Dimensions of hub and tarmac
    public static final Rotation2d centerLineAngle 
    = Rotation2d.fromDegrees(66.0);
    public static final Translation2d hubCenter 
    = new Translation2d(fieldLength / 2.0, fieldWidth / 2.0);
    public static final double tarmacInnerDiameter 
    = Utils.conversion_inchToMeters(219.25);
    public static final double tarmacOuterDiameter 
    = Utils.conversion_inchToMeters(237.31);
    public static final double tarmacFenderToTip 
    = Utils.conversion_inchToMeters(84.75);
    public static final double tarmacFullSideLength 
    = tarmacInnerDiameter * (Math.sqrt(2.0) - 1.0); 
    public static final double tarmacMarkedSideLength 
    = Utils.conversion_inchToMeters(82.83); 
    public static final double tarmacMissingSideLength 
    = tarmacFullSideLength - tarmacMarkedSideLength; 
    public static final double hubSquareLength 
    = tarmacOuterDiameter - (tarmacFenderToTip * 2.0);
    public static final Pose2d glassOrigin 
    = new Pose2d(0, -4.32, new Rotation2d()); 
    public static final Pose2d pwOrigin 
    = new Pose2d(0, -4.32, new Rotation2d());

    //Auto
    public static final double dumperTime(int value){
        final double dumperNearby = 1.5;
        final double dumperFar = 3.0;

        double[] _time = new double[2];
        _time[0] = dumperNearby;
        _time[1] = dumperFar;
    
        return _time[value];
    }

    //Auto
    public static final double pivotTime(int value){
        final double pivotShort = 1.5;
        final double pivotLong = 3.0;
    
        double[] _time = new double[2];
        _time[0] = pivotShort;
        _time[1] = pivotLong;
        
        return _time[value];
        }

}   

