// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.util;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    //Ultrasonic
    public static Ultrasonic higherUltrasonic;
    public static Ultrasonic lowerUltrasonic;

    public boolean enabling;

    public Sensors(){
        pigeon = new PigeonIMU(Constants.pigeonID);

        rightCANcoder = new CANCoder(Constants.rightCANcoderID);
        rightCANcoder.configFeedbackCoefficient(Constants.wheelPerimeter * Constants.degreeCoefficientCANcoder / 360 , "meter" , SensorTimeBase.PerSecond);

        leftCANcoder = new CANCoder(Constants.leftCANcoderID);
        leftCANcoder.configFeedbackCoefficient(Constants.wheelPerimeter * Constants.degreeCoefficientCANcoder / 360, "meter", SensorTimeBase.PerSecond);

        hangCANcoder = new CANCoder(Constants.hangCANcoderID);
        hangCANcoder.configFeedbackCoefficient(Constants.wheelPerimeter * Constants.degreeCoefficientCANcoder / 360, "meter", SensorTimeBase.PerSecond);

        enabling = true;

        higherUltrasonic = new Ultrasonic(Constants.higherUltrasonicPingChannel, Constants.higherUltrasonicEchoChannel);
        higherUltrasonic.setEnabled(enabling);

        lowerUltrasonic = new Ultrasonic(Constants.lowerUltrasonicPingChannel, Constants.lowerUltrasonicEchoChannel);
        lowerUltrasonic.setEnabled(enabling);

        Ultrasonic.setAutomaticMode(enabling);
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
     * 
     */
    public double getRightSpeed(){
        double rightSpeed = rightCANcoder.getVelocity();

        return rightSpeed;
    }

    public double getSpeed(){
        return (getLeftSpeed() + getRightSpeed())/ 2; 
    }

    /**
     * Encoder outputs using SmartDashboard
     */
    public void encoderOutputs(){
        SmartDashboard.putNumber("Left Speed", getLeftSpeed());
        SmartDashboard.putNumber("Right Speed", getRightSpeed());
        SmartDashboard.putNumber("Average Speed", getSpeed());
    }

    //GYRO
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

    //ULTRASONIC

    /**
     * Gets distance in centimeters of the target returned from the ultrasonic sensor.
     * @return distance
     */
    public static double getDistanceHigherCM(){
        double distance;
        //distance = Utils.conversion_inchToCM(higherUltrasonic.getRangeInches());
        distance = getDistanceHigherMM() / 10;

        return distance;

    }

    /**
     * Gets distance in millimeters of the target returned from the ultrasonic sensor.
     * @return distance
     */
    public static double getDistanceHigherMM(){
        double distance;
        //distance = Utils.conversion_inchToMM(higherUltrasonic.getRangeInches());
        distance = higherUltrasonic.getRangeMM();

        return distance;
    }  

 
    /**
     * Gets distance in millimeters of the target returned from the ultrasonic sensor.
     * @return distance
     */
    public static double getDistanceLowerCM(){
        double distance;
        //distance = Utils.conversion_inchToCM(higherUltrasonic.getRangeInches());
        distance = getDistanceLowerMM() / 10;

        return distance;
    }  

    /**
     * Gets distance in millimeters of the target returned from the ultrasonic sensor.
     * @return distance
     */
    public static double getDistanceLowerMM(){
        double distance;
        //distance = Utils.conversion_inchToMM(higherUltrasonic.getRangeInches());
        distance = lowerUltrasonic.getRangeMM();

        return distance;
    }

    /**
     * Checks if lower ball is detected or not
     * @return is lower cargo ball detected
     */ 
    public static boolean isLowerCargoDetected(){
        boolean state;
        state = (getDistanceLowerCM()) < (Constants.ultrasonicDistanceAcross - 5.5);

        return state;
    }

    /**
     * Checks if lower ball is detected or not
     * @return is higher cargo ball detected
     */
    public static boolean isHigherCargoDetected(){
        boolean state;
        state = (getDistanceHigherCM()) < (Constants.ultrasonicDistanceAcross - 5.5);

        return state;
    }

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
     * Ultrasonic sensor outputs using SmartDashboard
    */
    public void ultrasonicOutputs(){
        SmartDashboard.putNumber("Lower Sensor Output in MM", getDistanceLowerMM());
        SmartDashboard.putNumber("Lower Sensor Output in CM", getDistanceLowerCM());
        SmartDashboard.putNumber("Higher Sensor Output in MM", getDistanceHigherMM());
        SmartDashboard.putNumber("Higher Sensor Output in MM", getDistanceHigherMM());

        SmartDashboard.putBoolean("Is Lower Ball Detected", isLowerCargoDetected());
        SmartDashboard.putBoolean("Is Highter Ball Detected", isHigherCargoDetected());
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