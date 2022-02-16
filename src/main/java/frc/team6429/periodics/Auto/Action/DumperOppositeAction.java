// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.team6429.subsystems.Dumper;

/** Add your docs here. */
public class DumperOppositeAction implements Action{

    public Dumper mDumper;
    public Timer timer;
    public double wanted_time;
    public double wanted_rate;
    public double intake_speed;
    public double conveyor_speed;

    public DumperOppositeAction(double wantedRate, double wantedTime, double intakeSpeed, double conveyorSpeed){
        intake_speed = intakeSpeed;
        conveyor_speed = conveyorSpeed;
        wanted_rate = wantedRate;
        wanted_time = wantedTime;
        timer = new Timer();
        mDumper = Dumper.getInstance();
    }

    @Override
    public void start() {
        timer.reset();
        timer.start();
    }

    @Override
    public void update() {

        mDumper.dumperOppositeWithIndexer(wanted_rate, intake_speed, conveyor_speed);
    }

    @Override
    public boolean isFinished() {
    
        return timer.get() >= wanted_time;
    }

    @Override
    public void done() {
        mDumper.dumperStopWithIndexer();
        
    }
}
