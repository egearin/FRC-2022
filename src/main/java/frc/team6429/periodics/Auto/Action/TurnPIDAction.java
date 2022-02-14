// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.periodics.Auto.Action;

import frc.team6429.util.Sensors;
import frc.team6429.util.Utils;
import frc.team6429.subsystems.Drive;

/** Robot Tuen PID Action */
public class TurnPIDAction implements Action {

    public Drive mDrive;
    public Sensors mSensors;
    public double _desiredAngle;
    public double _tolerance;

    public TurnPIDAction(double desiredAngle, double tolerance) {
        mDrive = Drive.getInstance();
        _desiredAngle = desiredAngle;
        _tolerance = tolerance;
    }
    @Override
    public void start() {
    }

    @Override
    public void update() {
        mDrive.turnPID(_desiredAngle);
    }   

    @Override
    public boolean isFinished() {
        return Utils.tolerance(mSensors.getGyroAngle(), _desiredAngle, 1);
    }

    @Override
    public void done() {
        mDrive.stopDrive();
    }
}
