// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team6429.robot.RobotData;
import frc.team6429.robot.RobotData.FourCargoPaths;
import frc.team6429.robot.RobotData.PathType;
import frc.team6429.robot.RobotData.ThreeCargoPaths;
import frc.team6429.robot.RobotData.TwoCargoPaths;
import frc.team6429.subsystems.Drive;
import frc.team6429.util.Utils;

/** 
 * Improved version of FourDimensionalTrajectory.java 
 */
public class RemasteredFourDimensional implements Action {

    Drive mDrive;
    boolean finished = false;
    Trajectory trajectory;               
    FourCargoPaths fourCargoPaths;
    ThreeCargoPaths threeCargoPaths;
    TwoCargoPaths twoCargoPaths;
    PathType pathType;

    public RemasteredFourDimensional(Trajectory trajectory){
        mDrive = Drive.getInstance();
    }


    @Override
    public void start() {
        mDrive.stopDrive();
        mDrive.resetRamsete();
        //paths();
        Utils.printTrajectoriesToDashboard(List.of(trajectory), RobotData.fieldSim);
        mDrive.constructOdometry(mDrive.getFusedGyroRotation2D());
        mDrive.createRamseteManager(trajectory);
        mDrive.resetOdometry(trajectory.getInitialPose());
    }

    @Override
    public void update() {
        mDrive.updateOdometry();
        RobotData.fieldSim.setRobotPose(mDrive.getPose());
        SmartDashboard.putData(RobotData.fieldSim);
        finished = mDrive.followTrajectory();
    } 

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void done() {
        mDrive.stopDrive();
    }

    public void paths(){
        switch (pathType){
            case FOURCARGO:
                switch(fourCargoPaths){
                    case _01:
                        trajectory 
                        = RobotData.fourCargo.get(0);
                        break;
                    case _02:
                        trajectory 
                        = RobotData.fourCargo.get(1);
                        break;
                    case _03:
                        trajectory 
                        = RobotData.fourCargo.get(2);
                        break;
                    case _04:
                        trajectory
                         = RobotData.fourCargo.get(3);
                        break;
                    case _05:
                        trajectory 
                        = RobotData.fourCargo.get(4);
                        break;
                }
            break;

            case THREECARGO:
                switch(threeCargoPaths){
                    case _01:
                        trajectory 
                        = RobotData.threeCargo.get(0);
                        break;
                    case _02:
                        trajectory 
                        = RobotData.threeCargo.get(1);
                        break;
                    case _03:
                        trajectory 
                        = RobotData.threeCargo.get(2);
                        break;
                    case _04:
                        trajectory 
                        = RobotData.threeCargo.get(3);
                        break;
                } 
            break;

            case TWOCARGO:
                switch(twoCargoPaths){
                    case _01:
                        trajectory 
                        = RobotData.twoCargo.get(0);
                        break;
                    case _02:
                        trajectory 
                        = RobotData.twoCargo.get(1);
                        break;
                }
            break;

        }   
    }
}

