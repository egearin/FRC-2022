// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.team6429.subsystems.Drive;
import frc.team6429.util.Sensors;

/** Add your docs here. */
public class DriveAction implements Action{

    public Drive mDrive;
    public Sensors mSensors;
    public Timer timer;
    public double time;
    public double speed;
    public double rotation;

    public DriveAction(double wantedTime, double wantedSpeed, double wantedRotation){
        time = wantedTime;
        speed = wantedSpeed;
        rotation = wantedRotation;
        mDrive = Drive.getInstance();
        mSensors = Sensors.getInstance();
        timer = new Timer();
    }

    @Override
    public void start() {
        mSensors.resetCANcoder();
        timer.reset();
        timer.start();
    }

    @Override
    public void update() {
        mDrive.robotDrive(speed, rotation);
        
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= time;
    }

    @Override
    public void done() {
        mDrive.stopDrive();
    }
}
