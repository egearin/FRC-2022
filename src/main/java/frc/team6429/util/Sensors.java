// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.util;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import frc.team6429.robot.Constants;

/** Add your docs here. */
public class Sensors {

    private static Sensors mInstance = new Sensors();

    public static Sensors getInstance(){
        return mInstance;
    }
    //Pigeon
    public PigeonIMU pigeon;

    //CANcoder
    public CANCoder leftCANcoder;
    public CANCoder rightCANcoder;
    public CANCoder hangCANcoder;


    public Sensors(){
        pigeon = new PigeonIMU(Constants.pigeonID);

        rightCANcoder = new CANCoder(Constants.rightCANcoderID);
        rightCANcoder.configFeedbackCoefficient(Constants.wheelPerimeter * Constants.degreeCoefficientCANcoder / 360 , "meter" , SensorTimeBase.PerSecond);

        leftCANcoder = new CANCoder(Constants.leftCANcoderID);
        leftCANcoder.configFeedbackCoefficient(Constants.wheelPerimeter * Constants.degreeCoefficientCANcoder / 360, "meter", SensorTimeBase.PerSecond);

        hangCANcoder = new CANCoder(Constants.hangCANcoderID);
        hangCANcoder.configFeedbackCoefficient(Constants.wheelPerimeter * Constants.degreeCoefficientCANcoder / 360, "meter", SensorTimeBase.PerSecond);

    }

                //COLLECT DATA
    /**
     * Gets Gyro Angle
     */
    public double getGyroAngle(){
        return pigeon.getFusedHeading();
    }
    public boolean isPigeonReady(){
        PigeonState state = pigeon.getState();
        return state == PigeonState.Ready;
    }

    /**
     * Only Get Yaw Angle
     */
    public double getYawAngle(){
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[0];
    }

    /**
     * Only Get Left Speed
     * @return Left Speed
     */
    public double getLeftSpeed(){
        double leftSpeed = leftCANcoder.getVelocity();
    
        return leftSpeed;
    }

    /** 
     * Only Get Right Speed
     * 
     */
    public double getRightSpeed(){
        double rightSpeed = rightCANcoder.getVelocity();

        return rightSpeed;
    }

    public double getSpeed(){
        return (getLeftSpeed() + getRightSpeed())/ 2; 
    }


                //RESET

    /**
     * Resets all CANcoder values
     */
    public void resetCANcoder(){
        hangCANcoder.setPosition(0);
        leftCANcoder.setPosition(0);
        rightCANcoder.setPosition(0);
    }

    /**
     * Resets Gyro Values
     */
    public void gyroReset(){
        pigeon.setYaw(0);
        pigeon.setAccumZAngle(0);
        pigeon.setFusedHeading(0);
    }

    public void resetYawAngle(){
        pigeon.setYaw(0);
    }

    /**
     * Resets Sensors
     */
    public void resetSensors(){
        gyroReset();
        resetCANcoder();
    }

}