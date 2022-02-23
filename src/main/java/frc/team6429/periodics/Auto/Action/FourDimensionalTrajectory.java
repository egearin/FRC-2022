// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team6429.robot.RobotData;
import frc.team6429.subsystems.Drive;

/** Add your docs here. */
public class FourDimensionalTrajectory implements Action {

    Drive mDrive;
    boolean finished = false;
    int currentTrajectory;
    List<Trajectory> trajectories;

    public FourDimensionalTrajectory(){
        mDrive = Drive.getInstance();
        trajectories = new ArrayList<>();
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
    } 

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void done() {
        mDrive.stopDrive();
    }
}

