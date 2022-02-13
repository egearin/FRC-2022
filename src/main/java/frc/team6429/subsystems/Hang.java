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

    private boolean statusRear = false;
    private boolean statusFront = false;
    
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
    }

    public void simpleHangMotorUp(double speed){
        hangMotor.set(speed);
    }

    public void simpleHangMotorDown(double speed){
        hangMotor.set(-speed);
    }

    //to be filled
    public void hangPeriodic(){
        
    }
    public void hangStop(){
        hangMotor.stopMotor();
    }

    public void testLeftPiston(double wantedPressure){
        
        compressorLeft.enableDigital();
        if(compressorLeft.getPressure() >= wantedPressure){
            compressorLeft.disable();
            climbLeftSolenoid.set(true);
        }
    }

    public void testRightPiston(double wantedPressure){
        
        compressorLeft.enableDigital();
        if(compressorLeft.getPressure() >= wantedPressure){
            compressorLeft.disable();
            climbRightSolenoid.set(true);
        }
    }

    public void testForwardPiston(double wantedPressure){
        
        compressorForward.enableDigital();
        if(compressorForward.getPressure() >= wantedPressure){
            compressorForward.disable();
            climbForwardSolenoid.set(true);
        }
    }

    public void closeAllSolenoids(){
        climbLeftSolenoid.set(false);
        climbRightSolenoid.set(false);
        climbForwardSolenoid.set(false);
    }

    public double[] getPressureCompressors(){
        double[] pressureList = new double[3];
        pressureList[0] = compressorLeft.getPressure();
        pressureList[1] = compressorRight.getPressure();
        pressureList[2] = compressorForward.getPressure();

        return pressureList;
    }

    public void releaseRear(){
        climbLeftSolenoid.set(true);
        climbRightSolenoid.set(true);
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

    public void isimlazimbuna(){

        compressorLeft.enableDigital();
        compressorRight.enableDigital();
        compressorForward.enableDigital();

        double[] pressure = getPressureCompressors();

        if(pressure[0] == Constants.climbPressureRear && pressure[1] == Constants.climbPressureRear){
            compressorLeft.disable();
            compressorRight.disable();
            statusRear = true;
        }

        if(pressure[2] == Constants.climbPressureFront){
            compressorForward.disable();
            statusFront = true;
        }

    }


}
