// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import frc.team6429.util.Sensors;
import frc.team6429.util.Utils;
import frc.team6429.robot.Constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/** Climb subsystem class. */
public class Climb {

    private static Climb mInstance = new Climb();

    public static Climb getInstance(){
        return mInstance;
    }

    public PigeonIMU pigeon;
    public WPI_TalonFX hangMotor;
    public CANCoder hangCANcoder;
    public TalonFXConfiguration config;
    public Solenoid climbForwardSolenoid;
    public Sensors mSensor;
    public Drive mDrive;
    public Timer timer;
    
    private Climb(){
        hangMotor = Utils.makeTalonFX(Constants.hangMotorID, false);
        config = new TalonFXConfiguration();
        hangMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        hangMotor.setSensorPhase(false);
        hangMotor.setSelectedSensorPosition(0);
        pigeon = new PigeonIMU(Constants.pigeonID);
        mSensor = Sensors.getInstance();
        timer = new Timer();
        timer.start();
        /*hangCANcoder = new CANCoder(Constants.hangCANcoderID);
        hangMotor.configAllSettings(config);*/
    }
    
    //with time
    public void simpleHangMotorUp(double speed){
        hangMotor.set(speed);
    }
    //with time
    public void simpleHangMotorDown(double speed){
        hangMotor.set(-speed);
    }

    public void preClimbPeriodic(){
        mDrive.pto.set(true);
        climbForwardSolenoid.set(true);
        timer.reset();
        timer.start();
    }

    public boolean getAngleError(){
        double[] angles = new double[3];

        pigeon.getAccumGyro(angles);
        if(angles[1] == 0){
            return true;
        }
        else{
            return false;
        }
    }

    public void climbPeriodic(){
        timer.reset();
        timer.start();
        if(timer.get() <= Constants.climbTime){
            simpleHangMotorUp(Constants.climbSpeed);;
        }
        else{
            if(getAngleError()){
                climbForwardSolenoid.set(true);
            }
            else{
                simpleHangMotorDown(0.5);
            }
        }
    }

    public void forwardClimbPeriodic(){
        climbForwardSolenoid.set(true);
        
    }

    public void hangStop(){
       hangMotor.stopMotor();
    }

}
