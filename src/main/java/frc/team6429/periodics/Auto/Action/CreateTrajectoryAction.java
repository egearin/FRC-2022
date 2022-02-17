// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.trajectory.Trajectory;

import frc.team6429.robot.RobotData;
import frc.team6429.util.Utils;
import frc.team6429.subsystems.Drive;

/** This class is used for Creating Trajectory */
public class CreateTrajectoryAction implements Action {

    public PathType _pathType;
    public Drive mDrive;
    public boolean finished;
    public List<Trajectory> trajectories;
    public int currentTrajectory;

    public enum PathType{
        TWOCARGO,
        THREECARGO,                                         
        FOURCARGO,
        FIVECARGO;
    }

    public CreateTrajectoryAction(PathType pathType){
        _pathType = pathType;
        mDrive = Drive.getInstance();
        RobotData.fieldSim = new Field2d();
        trajectories = new ArrayList<Trajectory>();
    }

    @Override
    public void start() {
        switch (_pathType){
            case TWOCARGO:
                trajectories = RobotData.listOfTrajectories.get(0);
                RobotData.selectedTrajectory = 0;
                break;
            case THREECARGO:
                trajectories = RobotData.listOfTrajectories.get(1);
                RobotData.selectedTrajectory = 1;
                break;
            case FOURCARGO:
                trajectories = RobotData.listOfTrajectories.get(2);
                RobotData.selectedTrajectory = 2;
                break;
            case FIVECARGO:
                trajectories = RobotData.listOfTrajectories.get(3);
                RobotData.selectedTrajectory = 3;
            default:
                trajectories = RobotData.listOfTrajectories.get(0);
                RobotData.selectedTrajectory = 2;
        }
         
        mDrive.stopDrive();
        mDrive.resetRamsete();
        Utils.printTrajectoriesToDashboard(trajectories, RobotData.fieldSim);
        mDrive.constructOdometry(mDrive.getFusedGyroRotation2D());
        mDrive.createRamseteManager(trajectories.get(0));
        mDrive.resetOdometry(trajectories.get(0).getInitialPose());
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {
    }
}
