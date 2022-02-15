// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.team6429.util.Utils;

/** 
 * All Robot Constants 
*/
public class Constants {

    //Drive Motor Ports with "TalonFX"
    public static final int rightOneMotorID = 26;
    public static final int rightTwoMotorID = 25;

    public static final int leftOneMotorID = 24;
    public static final int leftTwoMotorID = 23;

    //Other Subsystems Motor Ports and IDs
    public static final int intakeMotorID = 22;
    public static final int conveyorMotorID = 21;

    public static final int dumperMotorID = 20;

    //public static final int hangMotorPort = 0;
    public static final int hangMotorID = 19;

    //Solenoid Ports
    public static final int shifterChannel = 1; //Single Solenoid for REVPH
    public static final int ptoChannel = 3; //Single Solenoid for REVPH
    public static final int pivotPistonChannel = 14; //Single Solenoid for REVPH
    public static final int pivotPistonsForwardChannel = 13; //Double Solenoid for REVPH 
    public static final int pivotPistonsReverseChannel = 12; //Double Solenoid for REVPH
    public static final int climbLeftPistonChannel = 0;
    public static final int climbRightPistonChannel = 0;
    public static final int climbForwardPistonChannel = 0;

    public static final int shifter1Port = 0; //Single Solenoid for CTREPCM
    public static final int pto1Port = 1; //Single Solenoid for CTREPCM
    public static final int pivotPiston1Channel = 4; //Single Solenoid for CTREPCM
    public static final int pivotPistons1ForwardChannel = 2; //Double Solenoid for CTREPCM
    public static final int pivotPistons1ReverseChannel = 3; //Double Solenoid for CTREPCM


    //Pigeon port 
    public static final int pigeonID = 7;

    //CANdle Port
    public static final int candleID = 13;
    
    //PID Constants
    public static final double kDriveP = 0;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;

    //Feedforward Constants
    public static final double kDriveS = 0;
    public static final double kDriveV = 0;
    public static final double kDriveA = 0;

    public static final double maxVoltageInput = 12;

    //CANcoder Ports
    public static final int leftCANcoderID = 6;
    public static final int rightCANcoderID = 5;

    public static final int hangCANcoderID = 100;

    //Ultrasonic Constants
    public static final int trigPinHigh = 1;
    public static final int trigPinLow= 0;
    
    public static final int higherSensor = 1;
    public static final int lowerSensor = 0;
    
    //Drive Constants
    public static final double kTrackWidth = 0;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);
    public static final double kMaxSpeed = 0;
    public static final double kMaxAcceleration = 0;
    public static final double kRamseteB = 0;
    public static final double kRamseteZeta = 0;
    public static final double kTrajectoryP = 0;

    public static final double shifterPulseDuration = 1;
    public static final double ptoPulseDuration = 1;
    public static final double pivotPulseDuration = 1; //for Single Solenoid

    //Hub
    public static final int pdhID = 12; //REV Power Distribution Hub
    public static final int phID = 11;  //REV Pneumatics Hub

    //Gamepad and Drivepanel Constants
    public static final int gamepadJoystick = 0;
    public static final int panel1Joystick = 1;
    public static final int panel2Joystick = 2;

    //---Gamepad---
    public static final int axis_forward = 3;
    public static final int axis_reverse = 2;
    public static final int axis_steering = 0;
    public static final int axis_sensetiveSteering = 4;

    public static final int shifterButton = 6;
    public static final int shifterOneButton = 0;
    public static final int shifterTwoButton = 0;

    public static final int ptoButton = 5;

    public static final int customIndexerOnButtonGamepad = 0;
    public static final int customIndexerOffButtonGamepad = 0;

    public static final int intakeOnButtonGamepad = 0;
    public static final int intakeReverseButtonGamepad = 0;

    public static final int conveyorOnButtonGamepad = 0;
    public static final int conveyorReverseButtonGamepad = 0;

    public static final int dumperButtonGamepad = 0;
    public static final int dumperOppositeButtonGamepad = 0;

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

    public static final double degreeCoefficientCANcoder = 7.792562E-05;
    public static final double wheelPerimeter = 0.31918581360472299303;
    public static final double canCoderCPR = 4096;
    public static final double robotMass = 35;
    
    //Necessary Constants
    public static final double traversalAngle = 0;
    public static final double climbPressureRear = 0;
    public static final double climbPressureFront = 0;
    
    //Field dimensions
    public static final double fieldLength = Utils.conversion_inchToMeters(54.0 * 12.0);
    public static final double fieldWidth = Utils.conversion_inchToMeters(27.0 * 12.0);
    public static final double hangarLength = Utils.conversion_inchToMeters(128.75);
    public static final double hangarWidth = Utils.conversion_inchToMeters(116.0);

    //Dimensions of hub and tarmac
    public static final Rotation2d centerLineAngle = Rotation2d.fromDegrees(66.0);
    public static final Translation2d hubCenter = new Translation2d(fieldLength / 2.0, fieldWidth / 2.0);
    public static final double tarmacInnerDiameter = Utils.conversion_inchToMeters(219.25);
    public static final double tarmacOuterDiameter = Utils.conversion_inchToMeters(237.31);
    public static final double tarmacFenderToTip = Utils.conversion_inchToMeters(84.75);
    public static final double tarmacFullSideLength = tarmacInnerDiameter * (Math.sqrt(2.0) - 1.0); // If the tarmac formed a full octagon
    public static final double tarmacMarkedSideLength = Utils.conversion_inchToMeters(82.83); // Length of tape marking outside of tarmac
    public static final double tarmacMissingSideLength = tarmacFullSideLength - tarmacMarkedSideLength; // Length removed b/c of corner cutoff
    public static final double hubSquareLength = tarmacOuterDiameter - (tarmacFenderToTip * 2.0);
}   

