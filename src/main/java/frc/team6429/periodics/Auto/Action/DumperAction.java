// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.team6429.subsystems.Dumper;

/** Add your docs here. */
public class DumperAction implements Action{

    public Dumper mDumper;
    public Timer timer;
    public double wanted_time;
    public double wanted_rate;

    public DumperAction(double wantedRate, double wantedTime){
        mDumper = Dumper.getInstance();
        timer = new Timer();
        wanted_time = wantedTime;
        wanted_rate = wantedRate;
    }
    @Override
    public void start() {
        timer.reset();
        timer.start();
    }

    @Override
    public void update() {
        mDumper.dumperSendWithIndexer(wanted_rate);
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