// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.R4D9.Constants;
import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class Drive {

    private static Drive mInstance = new Drive();

    public static Drive getInstance(){
        return mInstance;
    }
    //Follower VictorSPX
    public WPI_VictorSPX driveLOne;
    public WPI_VictorSPX driveLTwo;
    public WPI_VictorSPX driveROne;
    public WPI_VictorSPX driveRTwo;
    
    //PID 
    public PIDController PID;
    public PIDController leftController = new PIDController(0, 0, 0);
    public PIDController rightController = new PIDController(0, 0, 0);
    
    //Encoders
    public Encoder driveLeftEnc;
    public Encoder driveRightEnc;

    //Sensors 
    public PigeonIMU pigeon;

    //Master VictorSPX
    public MotorControllerGroup leftMotor;
    public MotorControllerGroup rightMotor;

    public DifferentialDrive differentialDrive;
    public SimpleMotorFeedforward driveFeedforward;

    

    
    public Drive(){

        PID = new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);
        
        driveLOne = new WPI_VictorSPX(Constants.driveOneLeftMotorPort);
        driveLTwo = new WPI_VictorSPX(Constants.driveTwoLeftMotorPort);
        driveROne = new WPI_VictorSPX(Constants.driveOneRightMotorPort);
        driveRTwo = new WPI_VictorSPX(Constants.driveTwoRightMotorPort);

        leftMotor = new MotorControllerGroup(driveLOne, driveLTwo);
        rightMotor = new MotorControllerGroup(driveROne, driveRTwo);

        differentialDrive = new DifferentialDrive(leftMotor, rightMotor);

        pigeon = new PigeonIMU(Constants.pigeonPort);

        driveLeftEnc = new Encoder(Constants.driveLeftEncPort1, Constants.driveLeftEncPort2);
        driveLeftEnc.setSamplesToAverage(10);
        driveLeftEnc.setDistancePerPulse((1/Constants.EncoderCPR)* Constants.wheelPerimeter);

        driveRightEnc = new Encoder(Constants.driveRightEncPort1, Constants.driveRightEncPort2);
        driveRightEnc.setSamplesToAverage(10);
        driveRightEnc.setDistancePerPulse((1/Constants.EncoderCPR)* Constants.wheelPerimeter);
    
    }
    /**
     * Drive Outputs Using Smartdashboard
     */
    public void driveOutput(){

    }
    /**
     * Robot Drive Using Arcade Drive
     * @param speed
     * @param rotation_speed
     */
    public void robotDrive(double speed, double rotation_speed){
        robotDrive(speed, rotation_speed, 1);
    }

    /**
     * Robot Drive Using Arcade Drive
     * @param speed
     * @param rotation_speed
     * @param sensitivity
     */
    public void robotDrive(double speed, double rotation_speed, double sensivity){
        differentialDrive.arcadeDrive(speed, rotation_speed*sensivity);
    }

    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn){
        differentialDrive.curvatureDrive(speed, rotation, isQuickTurn);
    }
    
    /**
     * 
     * Gets Gyro Angle
     */
    public double getGyroAngle(){
        return pigeon.getFusedHeading();
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
     * Resets Encoders
     */
    public void encoderReset(){
        driveLeftEnc.reset();
        driveRightEnc.reset();
    }

    /**
     * Get Encoder Values
     */
    public void getDistanceLeft(){
        driveLeftEnc.getDistance();
    }

    public void getDistanceRight(){
        driveRightEnc.getDistance();
    }


    /**
    * Controls the left and right sides of the drive directly with voltages.
    *
    * @param leftVolts  the commanded left output
    * @param rightVolts the commanded right output
    */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotor.setVoltage(leftVolts);
        rightMotor.setVoltage(-rightVolts);
        differentialDrive.feed();
    }

    /**
     * Fully automated drive with PID
     * @param speed
     * @param rotation_angle
     */
    public void PIDDrive(double speed, double rotation_angle){
        double drivePID = PID.calculate(getGyroAngle(), rotation_angle);
        drivePID = MathUtil.clamp(drivePID, -1, 1);
        differentialDrive.curvatureDrive(speed, drivePID, false);
    }

    public void resetPID(){
        PID.reset();
    }

    /**
     * Resets everything resettable
     */
    public void reset(){
        resetPID();
        gyroReset();
        encoderReset();
    }

    public double simpleTurnPID(double desired_rotation){
        double rotation = 0;
        double kP = SmartDashboard.getNumber("Turn PID", 0.01);
        double minMax = SmartDashboard.getNumber("Min PID", 0.3);
        if (desired_rotation > 1.0){
                rotation = kP*desired_rotation + minMax;
        }
        else if (desired_rotation < 1.0){
                rotation = kP*desired_rotation - minMax;
        }
        return rotation;
    }

    public double turnPID(double desired_rotation){
        double rotation = 0;
        double kP = SmartDashboard.getNumber("Turn PID", 0.1);
        double minMax = 1.9;
        if (desired_rotation > 1.0){ // to the right
                rotation = kP*desired_rotation + minMax;
        }
        else if (desired_rotation < 1.0){ // to the left
                rotation = kP*desired_rotation - minMax;
        }
        tankDriveVolts(rotation, -rotation);
        return rotation;
    }
    /**
     * Stop Driving Robot
     */
    public void stopDrive(){
        differentialDrive.tankDrive(0, 0);
    }
}
