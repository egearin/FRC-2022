// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

/** Add your docs here. */
public abstract class InstaRunAction implements Action{

    @Override
    public void start() {
        instaRun();
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {
    }

    public abstract void instaRun();

}

