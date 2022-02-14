// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.team6429.util.Utils;
import frc.team6429.periodics.Auto.Action.CreateTrajectoryAction.PathType;
import frc.team6429.subsystems.Drive;

/** Add your docs here. */
public class CreateAndFollowMultipleTrajectory implements Action {

    PathType _pathType;
    Drive mDrive;
    boolean finished;
    List<Trajectory> trajectories;
    List<String> trajectoryPaths;
    int currentTrajectory;
    Field2d fieldSim;

    public CreateAndFollowMultipleTrajectory(PathType pathType){
        _pathType = pathType;
        mDrive = Drive.getInstance();
        fieldSim = new Field2d();
        trajectories = new ArrayList<Trajectory>();
        trajectoryPaths = new ArrayList<String>();
    }

    @Override
    public void start() {
        switch (_pathType){
            case TWOCARGO:
                trajectoryPaths.add("");
                break;
            case THREECARGO:
                trajectoryPaths.add("");
                break;
            case FOURCARGO:
                trajectoryPaths.add("");
                break;
            default:
                trajectoryPaths.add("");
        }
        mDrive.stopDrive();
        mDrive.resetRamsete();
        for(String trajectoryPath:trajectoryPaths){
            trajectories.add(Utils.readTrajectoryFromPW(trajectoryPath));
        }
        Utils.printTrajectoriesToDashboard(trajectories, fieldSim);
        mDrive.constructOdometry(mDrive.getFusedGyroRotation2D());
        mDrive.createRamseteManager(trajectories.get(0));
        mDrive.resetOdometry(trajectories.get(0).getInitialPose());
        finished = false;
        currentTrajectory = 0;
    }

    @Override
    public void update() {
        mDrive.updateOdometry();
        fieldSim.setRobotPose(mDrive.getPose());
        SmartDashboard.putData(fieldSim);
        finished = mDrive.followTrajectory();
        if (finished){
            currentTrajectory++;
            if(currentTrajectory < trajectories.size()){
                finished = false;
                mDrive.createRamseteManager(trajectories.get(currentTrajectory));
            }
        }
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
