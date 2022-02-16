// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import frc.team6429.util.Sensors;
//import frc.team6429.subsystems.Drive2;
import frc.team6429.util.Utils;
import frc.team6429.robot.Constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Hang {

    private static Hang mInstance = new Hang();

    public static Hang getInstance(){
        return mInstance;
    }

    public PigeonIMU pigeon;
    public WPI_TalonFX hangMotor;
    public CANCoder hangCANcoder;
    public TalonFXConfiguration config;

    // Solenoids for climb
    public Solenoid climbLeftSolenoid;
    public Solenoid climbRightSolenoid;
    public Solenoid climbForwardSolenoid;

    // Compressor for climb
    public Compressor compressorLeft;
    public Compressor compressorRight;
    public Compressor compressorForward;

    public Sensors mSensor;
    public Drive mDrive;

    public Timer timer;
    
    public Hang(){
        hangMotor = Utils.makeTalonFX(Constants.hangMotorID, false);
        hangMotor.configAllSettings(config);
        hangMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        hangMotor.setSensorPhase(false);
        hangMotor.setSelectedSensorPosition(0);

        hangCANcoder = new CANCoder(Constants.hangCANcoderID);

        pigeon = new PigeonIMU(Constants.pigeonID);

        mDrive = new Drive();

        climbLeftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.climbLeftPistonChannel);
        climbRightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.climbRightPistonChannel);
        climbForwardSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.climbForwardPistonChannel);   
        
        compressorLeft = new Compressor(PneumaticsModuleType.REVPH);
        compressorRight = new Compressor(PneumaticsModuleType.REVPH);
        compressorForward = new Compressor(PneumaticsModuleType.REVPH);

        mSensor = new Sensors();

        timer = new Timer();
        timer.start();
    }

    public void simpleHangMotorUp(double speed){
        hangMotor.set(speed);
    }

    public void simpleHangMotorDown(double speed){
        hangMotor.set(-speed);
    }

    public void preClimbPeriodic(){
        mDrive.pto.set(true);
        climbForwardSolenoid.set(false);
        timer.reset();
        timer.start();
    }

    //to be filled
    public void hangPeriodic(double speed, double wantedTime){

        preClimbPeriodic();
    
        if(timer.get() <= wantedTime){
            mDrive.robotDrive(-speed, 0);
            //değerler ters mi emin değilim
            compressAir();
        }
        else if(ifInPosition()){
            if((Constants.climbPressureFront - Constants.pressureTolerance) <= compressorForward.getPressure() && 
            (Constants.climbPressureFront + Constants.pressureTolerance) >= compressorForward.getPressure()){
                climbForwardSolenoid.set(true);
            }
            else{
                compressAir();
            }
        }
        else{
            mDrive.robotDrive(speed, 0);
            //kanca kodu gelmesi gerekiyorsa gelsin
        }
    }

    public void hangStop(){
        hangMotor.stopMotor();
    }

    public void testForwardPiston(double wantedPressure){
        
        compressorForward.enableDigital();
        if(compressorForward.getPressure() >= wantedPressure){
            compressorForward.disable();
            climbForwardSolenoid.set(true);
        }
    }

    public boolean ifInPosition(){
        if(mSensor.getZ() <= 1.0){
            if(mSensor.getZ() >= -1.0){
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return false;
        }
    }

    public void compressAir(){
        compressorForward.enableDigital();
        if(compressorForward.getPressure() == Constants.climbPressureFront){
            compressorForward.disable();
        }
        else{

        }
    }

    public boolean checkFrontPressure(){
        if(compressorForward.getPressure() == Constants.climbPressureFront){
            return true;
        }
        else{
            return false;
        }
    }

    public void activateFrontArm(){
        if(checkFrontPressure()){
            climbForwardSolenoid.set(true);
        }
        else{

        }
    }


}
