// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.team6429.robot.Constants;
import frc.team6429.util.Sensors;

/** 
 * New proposed version of robot "Climb Subsystem" class.
*/
public class ClimbRemastered {

    private static ClimbRemastered mInstance = new ClimbRemastered();

    public static ClimbRemastered getInstance(){
        return mInstance;
    }

    //TalonFX
    public WPI_TalonFX hangMotor;

    //Solenoid
    public Solenoid midRungLock;
    public Solenoid frictionBrake;

    //Other Subsystems
    public Drive mDrive;
    public Sensors mSensors;

    //Other
    public Timer timer;

    //Pigeon
    public PigeonIMU pigeon;

    //Configuration
    public TalonFXConfiguration config;

    /**
     * Climb Initialization
     */
    private ClimbRemastered(){
        mDrive = Drive.getInstance();
        pigeon = new PigeonIMU(Constants.pigeonID);
        hangMotor = new WPI_TalonFX(Constants.hangMotorID);
        hangMotor.setNeutralMode(NeutralMode.Brake);
        hangMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        //midRungLock = new Solenoid(Constants.phID, PneumaticsModuleType.REVPH, Constants.midRungLockChannel);
        frictionBrake = new Solenoid(Constants.phID, PneumaticsModuleType.REVPH, Constants.frictionBrakeChannel);
        config = new TalonFXConfiguration();
        mSensors = Sensors.getInstance();
        timer = new Timer();
    }  

    /**
     * Robot Using Friction Brake
     * @param state
     */
    public void brake(boolean state){
        frictionBrake.set(state);
    }

    /**
     * pre-climb
     * @return
     */
    public boolean climb(){
        boolean isInitialized;
        if(mDrive.pto.get()){
            isInitialized = true;
        }
        else{
            isInitialized = false;
        }

        return isInitialized;
    }
    
    /**
     * Auto-climb initialize period
     */
    public void climbInit(){
        mDrive.pto.set(true);
        midRungLock.set(false);
        timer.reset();
        timer.start();
    }

    /**
     * Auto-climb periodic
     */
    public void climbPeriodic(){
        if(timer.get() <= 1.5){
            mDrive.robotDrive(1, 0);
        }

        else{
            midRungLock.set(true);
            mDrive.stopDrive();
        }
    }

    public void traverse(double traversalPosition){
        if(hangMotor.getSelectedSensorPosition() <= traversalPosition){
            hangMotor.set(1);
        }

        else{
            brake(true);
            hangMotor.stopMotor();
        }
    }
}
