// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;

import frc.team6429.robot.RobotData;
import frc.team6429.subsystems.Drive;

/** This class is used for Following Trajectory */
public class FollowTrajectoryAction implements Action {

    Drive mDrive;
    boolean finished = false;
    int currentTrajectory;
    List<Trajectory> trajectories;

    public FollowTrajectoryAction(){
        mDrive = Drive.getInstance();
        trajectories = new ArrayList<>();
    }

    @Override
    public void done() {
        mDrive.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void start() {
        currentTrajectory = 0;
        trajectories = RobotData.listOfTrajectories.get(RobotData.selectedTrajectory);
    }

    @Override
    public void update() {
        mDrive.updateOdometry();
        RobotData.fieldSim.setRobotPose(mDrive.getPose());
        SmartDashboard.putData(RobotData.fieldSim);
        finished = mDrive.followTrajectory();
        if (finished){
            currentTrajectory++;
            if(currentTrajectory < trajectories.size()){
                finished = false;
                mDrive.createRamseteManager(trajectories.get(currentTrajectory));
            }
        }
    }
   
}
