// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Modes;

import frc.team6429.periodics.Auto.AutoModeEndedException;
import frc.team6429.periodics.Auto.Action.CreateTrajectoryAction;
import frc.team6429.periodics.Auto.Action.FollowTrajectoryAction;
import frc.team6429.periodics.Auto.Action.SeriesAction;
import frc.team6429.periodics.Auto.Action.CreateTrajectoryAction.PathType;

/** Add your docs here. */
public class ThreeCargoAuto extends AutoModeBase{

    public PathType _path;

    public ThreeCargoAuto(PathType path){
        _path =  path;
    }
    
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SeriesAction(new CreateTrajectoryAction(_path),
                                    new FollowTrajectoryAction()));
        
    }
}
