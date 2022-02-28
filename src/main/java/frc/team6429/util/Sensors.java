// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.util;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.ctre.phoenix.sensors.PigeonIMU_ControlFrame;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.team6429.robot.Constants;
import frc.team6429.subsystems.Drive;
import frc.team6429.subsystems.Climb;

import java.util.ArrayList;

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
    //public CANCoder hangCANcoder;

    //Ultrasonic
    /*public static DigitalOutput ultrasonicTriggerPinLow;
    public static DigitalOutput ultrasonicTriggerPinHigh; 
    public static AnalogInput lowerUltrasonicSensor;
    public static AnalogInput higherUltrasonicSensor;*/

    //Beam Sensor
    public DigitalInput beamSensorHigh;
    public DigitalInput beamSensorLow;

    //Other Subsystems
    public Drive mDrive;
    public Climb mHang;

    public static final String smart_dashboard = "SmartDashboard";

    private Sensors(){
        pigeon = new PigeonIMU(Constants.pigeonID);

        rightCANcoder = new CANCoder(Constants.rightCANcoderID);
        rightCANcoder.configFeedbackCoefficient(Constants.wheelPerimeter * Constants.degreeCoefficientCANcoder / 360 , "meter" , SensorTimeBase.PerSecond);
        
        leftCANcoder = new CANCoder(Constants.leftCANcoderID);
        leftCANcoder.configFeedbackCoefficient(Constants.wheelPerimeter * Constants.degreeCoefficientCANcoder / 360, "meter", SensorTimeBase.PerSecond);

        //ultrasonicTriggerPinHigh = new DigitalOutput(Constants.trigPinHigh);
        //ultrasonicTriggerPinLow = new DigitalOutput(Constants.trigPinLow);

        //lowerUltrasonicSensor = new AnalogInput(Constants.lowerSensor);
        //higherUltrasonicSensor = new AnalogInput(Constants.higherSensor);

        beamSensorHigh = new DigitalInput(Constants.higherBeamChannel);
        beamSensorLow = new DigitalInput(Constants.lowerBeamChannel);
        //hangCANcoder = new CANCoder(Constants.hangCANcoderID);
    }

    //----------ENCODER----------
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
     * @return Right Speed
     */
    public double getRightSpeed(){
        double rightSpeed = rightCANcoder.getVelocity();

        return rightSpeed;
    }

    /**
     * Get Average Speed
     * @return Speed
     */
    public double getSpeed(){
        return (getLeftSpeed() + getRightSpeed())/ 2; 
    }

    /**
     * Only Get Left Speed from Falcon-Integrated Sensor
     * @return Left Speed
     */
    public double getIntegratedLeftSpeed(){
        double leftSpeed = mDrive.driveLeftMotor.getSelectedSensorVelocity();

        return leftSpeed;
    }

    /**
     * Only Get Right Speed from Falcon-Integrated Sensor
     * @return Right Speed
     */
    public double getIntegratedRightSpeed(){
        double rightSpeed = mDrive.driveRightMotor.getSelectedSensorVelocity();

        return rightSpeed;
    }   

    /**
     * Get Average Speed from Falcon-Integrated Sensors
     * @return Speed
     */
    public double getIntagratedSpeed(){
        return (getIntegratedLeftSpeed() + getIntegratedLeftSpeed()) / 2;
    }

    /**
     * Only Get Left Position
     * @return CANcoder Left Position
     */
    public double getLeftPosition(){
        double leftPosition = leftCANcoder.getPosition();

        return leftPosition;
    }

    /**
     * Only Get Right Position
     * @return CANcoder Right Position
     */
    public double getRightPosition(){
        double rightPosition = rightCANcoder.getPosition();

        return rightPosition;
    }

    /**
     * Get Average Position
     * @return position
     */
    public double getPosition(){
        return (getLeftPosition() + getRightPosition()) / 2;
    }

    /**
     * Only Get Left Position from Falcon-Integrated Sensor
     * @return Left Position
     */
    public double getIntegratedLeftPos(){
        double leftPos = mDrive.driveRightMotor.getSelectedSensorPosition();

        return leftPos;
    }
    
    /**
     * Only Get Right Position from Falcon-Integrated Sensor
     * @return Right Position
     */
    public double getIntegratedRightPos(){
        double rightPos = mDrive.driveLeftMotor.getSelectedSensorPosition();

        return rightPos;
    }

    /**
     * Get Integrated Position from Falcon-Integrated Sensors
     * @return position
     */
    public double getIntagratedPosition(){
        return (getIntegratedLeftPos() + getIntegratedRightPos()) / 2;
    }

    /**
     * Get Output Position from Falcon-Integrated Sensor 
     * @return position
     */
    public double getHangPos(){
        double position = mHang.hangMotor.getSelectedSensorPosition();

        return position;
    }
    /**
     * Encoder outputs using SmartDashboard
     */
    public void encoderOutputs(){
        SmartDashboard.putNumber("Left Speed", getLeftSpeed());
        SmartDashboard.putNumber("Right Speed", getRightSpeed());
        SmartDashboard.putNumber("Average Speed", getSpeed());

        SmartDashboard.putNumber("Left F500 Speed", getIntegratedLeftSpeed());
        SmartDashboard.putNumber("Right F500 Speed", getIntegratedRightSpeed());
        SmartDashboard.putNumber("Average F500 Speed", getIntagratedSpeed());
    }

    //----------PIGEON----------
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
     * @return yaw
     */    
    public double getYawAngle(){
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);

        return ypr[0];
    }   

    /**
     * Only Get Pitch Angle
     * @return pitch
     */
    public double getPitchAngle(){
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);

        return ypr[1];
    }

    /**
     * Only Get Roll Angle
     * @return roll
     */
    public double getRollAngle(){
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);

        return ypr[2];
    }
    
    /**
     * Alternate Only Get Yaw Angle
     * @return yaw
     */
    public double getYaw(){
        double yaw;
        yaw = pigeon.getYaw();

        return yaw;
    }

    /**
     * Alternate Only Get Pitch Angle
     * @return pitch
     */
    public double getPitch(){
        double pitch;
        pitch = pigeon.getPitch();

        return pitch;
    }
    
    /**
     * Alternate Only Get Roll Angle
     * @return roll
     */
    public double getRoll(){
        double roll;
        roll = pigeon.getRoll();
    
        return roll;
    }

    /**
     * Gets Raw X angle in degrees per second
     * @return x axis
     */
    public double getX(){
        double[] xyz = new double[3];
        pigeon.getRawGyro(xyz);

        return xyz[0];
    }
    
    /**
     * Gets Raw Y angle in degrees per second
     * @return y axis
     */
    public double getY(){
        double[] xyz = new double[3];
        pigeon.getRawGyro(xyz);

        return xyz[1];
    }

    /**
     * Gets Raw Z angle in degrees per second
     * @return z axis
     */
    public double getZ(){
        double[] xyz = new double[3];
        pigeon.getRawGyro(xyz);

        return xyz[2];
    }

    /**
     * Pigeon Calibration Mode
     * @return CalibrationMode BootTareGyroAccel
     */
    public CalibrationMode calibrationMode(){
        pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);

        return PigeonIMU.CalibrationMode.valueOf(0);
    }

    public void pigeonOutput(){
        SmartDashboard.putNumber("getX", getX());
        SmartDashboard.putNumber("getY", getY());
        SmartDashboard.putNumber("getZ", getZ());
        SmartDashboard.putNumber("getRoll", getRoll());
        SmartDashboard.putNumber("getPitch", getPitch());
        SmartDashboard.putNumber("getYaw", getYaw());
        SmartDashboard.putNumber("getRollAngle", getRollAngle());
        SmartDashboard.putNumber("getPitchAngle", getPitchAngle());
        SmartDashboard.putNumber("getYawAngle", getYawAngle());
        SmartDashboard.putNumber("getGyroAngle", getGyroAngle());
    }   

    //----------ULTRASONIC----------
    /**
     * Sets lower ultrasonic sensor to on and higher ultrasonic sensor to off
     */
    public void turnOnLowerUltrasonic(){
        lowerOn();
        higherOff();
    }

    /**
     * Sets higher ultrasonic to on and lower ultrasonic to off
     */
    public void turnOnHigherUltrasonic(){
        higherOn();
        lowerOff();
    }

    /**
     * Sets both sensors to on
     */
    public void turnOnBothSensors(){
        turnOnHigherUltrasonic();
        turnOnLowerUltrasonic();
    }

    /**
     * Sets both sensors to off
     */
    public static void turnOffBothSensors(){
        higherOff();
        lowerOff();
    }
    
    /**
     * Only sets higher ultrasonic sensor to on
     */
    public static void higherOn(){
        //ultrasonicTriggerPinHigh.set(true);
    }

    /**
     * Only sets lower ultrasonic sensor to on
     */
    public static void lowerOn(){
        //ultrasonicTriggerPinLow.set(true);
    }

    /**
     * Only sets higher ultrasonic sensor to off
     */
    public static void higherOff(){
        //ultrasonicTriggerPinHigh.set(false);
    }

    /**
     * Only sets lower ultrasonic sensor to off
     */
    public static void lowerOff(){
        //ultrasonicTriggerPinHigh.set(false);
    }
    
    /**
     * Gets distance in centimeters of the target returned from the higher ultrasonic sensor.
     * @return higherMeasuredDistance
     */
    /*public double getDistanceHigherCM(){
        double higherMeasuredDistance;
        double voltageScaleFactor;
        higherOn();
        voltageScaleFactor = 5 / (RobotController.getVoltage5V());
        higherMeasuredDistance = (higherUltrasonicSensor.getValue()) * (voltageScaleFactor) * 0.125;

        return higherMeasuredDistance;
    }*/

    /**
     * Gets distance in millimeters of the target returned from the higher ultrasonic sensor.
     * @return higherDistanceMM
     */
    /*public double getDistanceHigherMM(){
        double higherDistanceMM;
        higherOn();
        higherDistanceMM = getDistanceHigherCM() * 10;

        return higherDistanceMM;
    } */

    /**
     * Gets distance in centimeters of the target returned from the lower ultrasonic sensor.
     * @return lowerMeasuredDistance
     */
    /*public double getDistanceLowerCM(){
        double lowerMeasuredDistance;
        double voltageScaleFactor;
        lowerOn();
        voltageScaleFactor = 5 / (RobotController.getVoltage5V());
        lowerMeasuredDistance = (lowerUltrasonicSensor.getValue()) * (voltageScaleFactor) * 0.125;

        return lowerMeasuredDistance;
    } */

    /**
     * Gets distance in millimeters of the target returned from the lower ultrasonic sensor.
     * @return lowerDistanceMM
     */
    /*public double getDistanceLowerMM(){
        double lowerDistanceMM;
        lowerOn();
        lowerDistanceMM = getDistanceLowerCM() * 10;

        return lowerDistanceMM;
    }*/

    /*public double getBallCount(){
        double status = 0;
        turnOnBothSensors();
        if(isHigherCargoDetected()){
            if(isLowerCargoDetected()){
                status = 2;
            }
            else{
                status = 1;
            }

        if(!isHigherCargoDetected()){
            status = 0;
            }

        }  

        return status;
    }*/

    public static enum UltrasonicStates{
        DEFAULT(false),
        BALLDETECTED(true);

    public final boolean states;

    UltrasonicStates(boolean isStates){
        states = isStates;
    }
    
    public boolean isStates(){
        return states;
      }
    }

    /**
     * Checks if lower ball is detected or not
     * @return is lower cargo ball detected
     */ 
    /*public boolean isLowerCargoDetected(){
        boolean state;
        lowerOn();

        state = (getDistanceLowerCM()) < (Constants.ultrasonicDistanceAcross - Constants.subtractedDistance(1));
        //state = (getDistanceLowerCM()) < (Constants.ultrasonicDistanceAcross - Constants.subtractedDistance(0));
        //state = (getDistanceLowerCM()) < (Constants.ultrasonicDistanceAcross - Constants.subtractedDistance(2));

        return state;
    }*/

    /**
     * Checks if lower ball is detected or not
     * @return is higher cargo ball detected
     */
    /*public boolean isHigherCargoDetected(){
        boolean state;
        higherOn();
        
        state = (getDistanceHigherCM()) < (Constants.ultrasonicDistanceAcross - Constants.subtractedDistance(1));
        //state = (getDistanceHigherCM()) < (Constants.ultrasonicDistanceAcross - Constants.subtractedDistance(0));
        //state = (getDistanceHigherCM()) < (Constants.ultrasonicDistanceAcross - Constants.subtractedDistance(2));

        return state;
    }*/

    /** 
    * Ultrasonic sensor outputs using SmartDashboard
    */
    public void ultrasonicOutputs(){
        /*SmartDashboard.putNumber("Lower Sensor Output in MM", getDistanceLowerMM());
        SmartDashboard.putNumber("Lower Sensor Output in CM", getDistanceLowerCM());
        SmartDashboard.putNumber("Higher Sensor Output in MM", getDistanceHigherMM());
        SmartDashboard.putNumber("Higher Sensor Output in MM", getDistanceHigherMM());

        SmartDashboard.putBoolean("Is Lower Ball Detected", isLowerCargoDetected());
        SmartDashboard.putBoolean("Is Highter Ball Detected", isHigherCargoDetected());*/
    }

    // -------BEAM SENSOR-------
    public boolean detectHigherCargo(){
        boolean state;
        state = beamSensorHigh.get();

        return state;
    }

    public boolean detectLowerCargo(){
        boolean state;
        state = beamSensorLow.get();

        return state;
    }

    public double ballCounter(){
        double status = 0;

        if(detectHigherCargo()){
            if(detectLowerCargo()){
                status = 2;
            }
            else{
                status = 1;
            }

        if(!detectHigherCargo()){
            status = 0;
            }

        }  

        return status;
    }

    public static enum BeamStates{
        DEFAULT(false),
        BALLDETECTED(true);

    public final boolean states;

    BeamStates(boolean isStates){
        states = isStates;
    }
    
    public boolean isStates(){
        return states;
      }
    }

    //----------RESET----------
    /**
     * Resets all CANcoder values
     */
    public void resetCANcoder(){
        //hangCANcoder.setPosition(0);
        leftCANcoder.setPosition(0);
        rightCANcoder.setPosition(0);
    }   

    /**
     * Resets all drive CANcoder values
     */
    public void resetDrive(){
        resetLeftCANcoder();
        resetRightCANcoder();
    }

    public void resetLeftCANcoder(){
        leftCANcoder.setPosition(0);
    }

    public void resetRightCANcoder(){
        rightCANcoder.setPosition(0);
    }

    /**
     * Resets all drive-integrated sensor values
     */
    public void resetDriveEnc(){
        resetLeftEnc();
        resetRightEnc();
    }

    /**
     * Resets left Falcon-integrated sensor value
     */
    public void resetLeftEnc(){
        mDrive.driveLeftMotor.setSelectedSensorPosition(0);
    }

    /**
     * Resets right Falcon-integrated sensor value
     */
    public void resetRightEnc(){
        mDrive.driveRightMotor.setSelectedSensorPosition(0);
    }

    /** 
     * Resets hang-integrated sensor values
    */
    public void resetHangEnc(){
        mHang.hangMotor.setSelectedSensorPosition(0);
    }

    /**
     * Resets Gyro Values
     */
    public void gyroReset(){
        pigeon.setYaw(0);
        pigeon.setAccumZAngle(0);
        pigeon.setFusedHeading(0);
    }

    /**
     * Resets Yaw Angle
     */
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