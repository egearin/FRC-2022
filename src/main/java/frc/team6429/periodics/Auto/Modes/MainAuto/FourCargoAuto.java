// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Modes.MainAuto;

import java.util.Arrays;

import org.ejml.ops.ReadCsv;

import frc.team6429.periodics.Auto.AutoModeEndedException;
import frc.team6429.periodics.Auto.Action.DumperAction;
import frc.team6429.periodics.Auto.Action.FourDimensionalTrajectory;
import frc.team6429.periodics.Auto.Action.NoopAction;
import frc.team6429.periodics.Auto.Action.ParallelAction;
import frc.team6429.periodics.Auto.Action.RemasteredFourDimensional;
import frc.team6429.periodics.Auto.Action.SeriesAction;
import frc.team6429.periodics.Auto.Action.StopAction;
import frc.team6429.periodics.Auto.Action.TrajectoryIndexerAction;
import frc.team6429.periodics.Auto.Action.CreateTrajectoryAction.PathType;
import frc.team6429.periodics.Auto.Action.StopAction.Stop;
import frc.team6429.periodics.Auto.Modes.AutoModeBase;
import frc.team6429.robot.Constants;
import frc.team6429.robot.Robot;
import frc.team6429.robot.RobotData;
import frc.team6429.robot.RobotData.InSync;
import frc.team6429.robot.Constants;
import frc.team6429.periodics.Auto.Action.FollowTrajectoryAction;
import frc.team6429.periodics.Auto.Action.CreateTrajectoryAction;

import edu.wpi.first.math.trajectory.Trajectory;

/** 
 * Main Four Cargo Autonomous Mode. 
 * This mode creates and follows trajectory paths. Ships four cargo ball to the hub.
*/
public class FourCargoAuto extends AutoModeBase{
    
    public PathType _path;

    public FourCargoAuto(PathType path){
        _path = path;
        
    }

    @Override
    protected void routine() throws AutoModeEndedException {

        
        /*runAction(new SeriesAction(Arrays.asList(
                        new ParallelAction(Arrays.asList(
                            new FourDimensionalTrajectory(),
                            new TrajectoryIndexerAction(1.5, InSync.INSYNC, 1, 0.75),
                                new SeriesAction(Arrays.asList(
                                    new FourDimensionalTrajectory(),
                                    new DumperAction(InSync.INSYNC, 1, 1, 1, 10),
                                        new ParallelAction(Arrays.asList(
                                            new FourDimensionalTrajectory(),
                                            new TrajectoryIndexerAction(1, InSync.INSYNC, 1, 1),
                                                new SeriesAction(Arrays.asList(
                                                    new FourDimensionalTrajectory(),
                                                    new DumperAction(InSync.INSYNC, 1, 1, 1, 2),
                                                        new FourDimensionalTrajectory()
                                                ))
                                        ))
                                ))
                        ))
                    ))
            );*/
        

        /*runAction(new SeriesAction(Arrays.asList(
                    new ParallelAction(Arrays.asList(
                        new REMASTERED_FourDimensionalTrajectory(RobotData.fourCargo.get(0)),
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
        );*/
        
        runAction(new SeriesAction(Arrays.asList(
                    new ParallelAction(Arrays.asList(
                        new RemasteredFourDimensional(RobotData.fourCargo.get(0)),
                        new TrajectoryIndexerAction(Constants.pivotTime(0), 1, 0.75)
                    )),
                    new SeriesAction(Arrays.asList(
                        new RemasteredFourDimensional(RobotData.fourCargo.get(1)),
                        new DumperAction(InSync.INSYNC, 1, 1, 1, Constants.dumperTime(0))
                    )),
                    new ParallelAction(Arrays.asList(
                        new RemasteredFourDimensional(RobotData.fourCargo.get(2)),
                        new TrajectoryIndexerAction(Constants.pivotTime(1),1, 1)
                    )),
                    new SeriesAction(Arrays.asList(
                        new RemasteredFourDimensional(RobotData.fourCargo.get(3)),
                        new DumperAction(InSync.INSYNC, 1, 1, 1, Constants.pivotTime(0))
                    )),
                    new ParallelAction(Arrays.asList(
                        new RemasteredFourDimensional(RobotData.fourCargo.get(4)),
                        new TrajectoryIndexerAction(Constants.pivotTime(0), 1, 1)
                    )),
                    new StopAction(1, Stop.ALL)
                ))
        );

        /*(new SeriesAction(Arrays.asList(
                        new ParallelAction(Arrays.asList(
                            new FourDimensionalTrajectory(),
                            new SeriesAction(Arrays.asList(
                                new TrajectoryIndexerAction(1.5, InSync.INSYNC, 1, 1),
                                new ParallelAction(Arrays.asList(
                                    new SeriesAction(Arrays.asList(
                                        new FourDimensionalTrajectory(),
                                        new NoopAction())),
                                new SeriesAction(Arrays.asList(
                                    new DumperAction(InSync.INSYNC, 1, 1, 1, 2.5),
                                    new NoopAction()
                )),
                    new ParallelAction(Arrays.asList(
                        new FourDimensionalTrajectory(),
                        new TrajectoryIndexerAction(2, InSync.INSYNC, 1, 1)
                )),
                    new ParallelAction(Arrays.asList(
                        new FourDimensionalTrajectory(),
                        new DumperAction(InSync.INSYNC, 1, 1, 1, 2.5)
                )),
                    new SeriesAction(Arrays.asList(
                        new FourDimensionalTrajectory(),
                        new StopAction(5, Stop.ALL)


                        ))
                    ))
                ))
            )))
        ));*/
    }
}
