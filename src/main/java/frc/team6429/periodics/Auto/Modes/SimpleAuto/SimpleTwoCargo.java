/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team6429.periodics.Auto.Modes.SimpleAuto;

import java.util.Arrays;

import frc.team6429.periodics.Auto.AutoModeEndedException;
import frc.team6429.periodics.Auto.Action.*;
import frc.team6429.periodics.Auto.Action.StopAction.Stop;
import frc.team6429.periodics.Auto.Modes.AutoModeBase;


/**
 * The robot initially has the cargo ball inside.
 * When the autonomous mode starts, the robot takes the second cargo ball in front of it and returns to its former position again.
 * Finally, it drops two cargo balls and goes out of line.
 */
public class SimpleTwoCargo extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SeriesAction(Arrays.asList(
                    new ParallelAction(
                        new PivotDownAction(),
                        new SeriesAction(Arrays.asList(
                            new ParallelAction(
                                new DriveAction(1, 1, 0),
                                new IndexerAction(1, 0.3)),

                    new SeriesAction(new DriveAction(1, -0.5, 0),
                                        new ParallelAction(
                                            new SeriesAction(new DumperOppositeAction(1, 1, 1, 1),
                                             new StopAction(1, Stop.Dumper)),
                    new SeriesAction(new DriveAction(1, 1, 0),
                                        new StopAction(1, Stop.Drive)
                    
                    )))
                ))           
            ))
        ));
    }
}
