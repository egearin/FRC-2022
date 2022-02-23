// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Modes.MainAuto;

import java.util.Arrays;

import edu.wpi.first.math.trajectory.Trajectory;

import frc.team6429.periodics.Auto.AutoModeEndedException;
import frc.team6429.periodics.Auto.Action.CreateTrajectoryAction;
import frc.team6429.periodics.Auto.Action.DumperAction;
import frc.team6429.periodics.Auto.Action.FollowTrajectoryAction;
import frc.team6429.periodics.Auto.Action.FourDimensionalTrajectory;
import frc.team6429.periodics.Auto.Action.ParallelAction;
import frc.team6429.periodics.Auto.Action.RemasteredFourDimensional;
import frc.team6429.periodics.Auto.Action.SeriesAction;
import frc.team6429.periodics.Auto.Action.TrajectoryIndexerAction;
import frc.team6429.periodics.Auto.Action.CreateTrajectoryAction.PathType;
import frc.team6429.periodics.Auto.Modes.AutoModeBase;
import frc.team6429.robot.RobotData;
import frc.team6429.robot.RobotData.InSync;

/** Add your docs here. */
public class AltFourCargo extends AutoModeBase{
    
    public PathType _path;

    public AltFourCargo(PathType path){
        _path = path;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // TODO Auto-generated method stub
        
    }

   /* @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SeriesAction(Arrays.asList(
                    new ParallelAction(Arrays.asList(
                        new REMASTERED_FourDimensionalTrajectory(),
                        new TrajectoryIndexerAction(1.5, InSync.INSYNC, 1, 0.75)
                    )),
                    new ParallelAction(Arrays.asList(
                        new REMASTERED_FourDimensionalTrajectory(RobotData.fourCargo.get(1)),
                        new DumperAction(InSync.INSYNC, 1, 1, 1, 10)
                    )),
                    new ParallelAction(Arrays.asList(
                        new REMASTERED_FourDimensionalTrajectory(RobotData.fourCargo.get(2)),
                        new TrajectoryIndexerAction(1, InSync.INSYNC, 1, 1)
                    )),
                    new ParallelAction(Arrays.asList(
                        new REMASTERED_FourDimensionalTrajectory(RobotData.fourCargo.get(3)),
                        new DumperAction(InSync.INSYNC, 1, 1, 1, 2),
                            new REMASTERED_FourDimensionalTrajectory(RobotData.fourCargo.get(4))
                    ))
                ))
        );
        
    }*/
}
