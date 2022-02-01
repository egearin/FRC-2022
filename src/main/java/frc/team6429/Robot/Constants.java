// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/** Constants */
public class Constants {

    //Drive Motor Ports with "VictorSPX"
    public static final int driveOneLeftMotorID = 1;
    public static final int driveTwoLeftMotorID = 2;
    public static final int driveOneRightMotorID = 3;
    public static final int driveTwoRightMotorID = 4;

    //Drive Motor Ports with "TalonFX"
    public static final int rightOneMotorID = 0;
    public static final int rightTwoMotorID = 0;

    public static final int leftOneMotorID = 0;
    public static final int leftTwoMotorID = 0;
    
    //Elevator Motor Ports 
    public static final int elevator1MotorPort = 1;
    public static final int elevator2MotorPort = 2;

    //Pigeon port 
    public static final int pigeonID = 1;
    
    //PID Constants
    public static final double kDriveP = 0;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;

    public static final double kElevatorP = 0;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;

    public static final double kDownwardsElevatorP = 0;
    public static final double kDownwardsElevatorI = 0;
    public static final double kDownwardsElevatorD = 0;
    
    public static final double kUpwardsElevatorP = 0;
    public static final double kUpwardsElevatorI = 0;
    public static final double kUpwardsElevatorD = 0;

    //Feedforward Constants
    public static final double kElevatorS = 0;
    public static final double kElevatorV = 0;
    public static final double kElevatorA = 0;

    public static final double kDownwardsElevatorS = 0;//Force needed to overcome the static friction
    public static final double kDownwardsElevatorV = 0;//how much voltage to input in order to cruise the motor,
    public static final double kDownwardsElevatorA = 0;
    public static final double kDownwardsElevatorG = 0;

    public static final double kUpwardsElevatorS = 0;
    public static final double kUpwardsElevatorV = 0;
    public static final double kUpwardsElevatorA = 0;
    public static final double kUpwardsElevatorG = 0;

    public static final double kDriveS = 0;
    public static final double kDriveV = 0;
    public static final double kDriveA = 0;

    public static final double maxVoltageInput = 12;
   

    //Encoder Ports
    public static final int elevatorEncPort1 = 10;
    public static final int elevatorEncPort2 = 11;
    
    public static final int driveLeftEncPort1 = 12;
    public static final int driveLeftEncPort2 = 13;

    public static final int driveRightEncPort1 = 14;
    public static final int driveRightEncPort2 = 15;

    //Encoder Directions
    public static boolean elevatorEncDirection = true;

    //CANcoder Ports
    public static final int driveLeftCANcoderID = 0;
    public static final int driveRightCANcoderID = 0;

    public static final int climbCANcoderID = 0;

    //Drive Constants
    public static final double kTrackWidth = 0;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);
    public static final double kMaxSpeed = 0;
    public static final double kMaxAcceleration = 0;
    public static final double kRamseteB = 0;
    public static final double kRamseteZeta = 0;
    public static final double kTrajectoryP = 0;
    //Gamepad Constants
    public static final int axis_forward = 3;
    public static final int axis_reverse = 2;
    public static final int axis_steering = 0;
    public static final int axis_sensetiveSteering = 4;

    //Distance Per Pulse Values 
    public static final double elevatorEncDistancePerPulse = 0;
    public static final double wheelPerimeter = 0.31918581360472299303;
    public static final double EncoderCPR = 360;


   
}

