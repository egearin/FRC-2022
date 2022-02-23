// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.team6429.robot.RobotData.DumperMode;
import frc.team6429.robot.RobotData.InSync;
import frc.team6429.subsystems.Dumper;

/** Add your docs here. */
public class DumperAction implements Action{

    public Dumper mDumper;
    public Timer timer;
    public double wanted_time;
    public double dumper_speed;
    public double intake_speed;
    public double conveyor_speed;
    public DumperMode dumperMode;
    public InSync inSync;

    public DumperAction(InSync inSync, double dumperSpeed, double intakeSpeed, double conveyorSpeed, double wantedTime){
        mDumper = Dumper.getInstance();
        timer = new Timer();
        wanted_time = wantedTime;
        dumper_speed = dumperSpeed;
        intake_speed = intakeSpeed;
        conveyor_speed = conveyorSpeed;
    }
    @Override
    public void start() {
        timer.reset();
        timer.start();
    }

    @Override
    public void update() {
        //mDumper.dumperSendWithIndexer(dumper_speed, intake_speed, conveyor_speed);
        mDumper.setSyncMode(inSync, dumper_speed, intake_speed, conveyor_speed);
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