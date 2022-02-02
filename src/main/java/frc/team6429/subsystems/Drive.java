// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team6429.robot.Constants;
import frc.team6429.util.Sensors;
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
    
    //Feedforward
    public SimpleMotorFeedforward driveFeedforward;

    //Sensors 
    public PigeonIMU pigeon;
    public CANCoder leftCANcoder;
    public CANCoder rightCANcoder;

    //Solenoid 
    public Solenoid shifter;

    //Master
    public MotorControllerGroup leftMotor;
    public MotorControllerGroup rightMotor;

    public DifferentialDrive chassis;
    
    //Odometry
    public DifferentialDriveOdometry odometry;
    public RamseteManager ramseteManager;

    

    
    public Drive(){

        PID = new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);

        driveFeedforward = new SimpleMotorFeedforward(Constants.kDriveS, Constants.kDriveV, Constants.kDriveA);
        
        driveLOne = new WPI_VictorSPX(Constants.driveOneLeftMotorID);
        driveLTwo = new WPI_VictorSPX(Constants.driveTwoLeftMotorID);

        driveROne = new WPI_VictorSPX(Constants.driveOneRightMotorID);
        driveRTwo = new WPI_VictorSPX(Constants.driveTwoRightMotorID);

        leftMotor = new MotorControllerGroup(driveLOne, driveLTwo);
        leftMotor.setInverted(false);
        
        rightMotor = new MotorControllerGroup(driveROne, driveRTwo);
        rightMotor.setInverted(true);

        chassis = new DifferentialDrive(leftMotor, rightMotor);

        pigeon = new PigeonIMU(Constants.pigeonID);

        shifter = new Solenoid(PneumaticsModuleType.REVPH,Constants.shifterPort);
        shifter.setPulseDuration(Constants.shifterPulseDuration);
        
        leftCANcoder = new CANCoder(Constants.leftCANcoderID);
        leftCANcoder.configFeedbackCoefficient(Constants.wheelPerimeter * Constants.degreeCoefficientCANcoder / 360, "meter", SensorTimeBase.PerSecond);

        rightCANcoder = new CANCoder(Constants.rightCANcoderID);
        rightCANcoder.configFeedbackCoefficient(Constants.wheelPerimeter * Constants.degreeCoefficientCANcoder / 360, "meter", SensorTimeBase.PerSecond);
        

    
    }

    private WPI_VictorSPX makeVictorSPX(int id , boolean invert) { 
        WPI_VictorSPX victorSPX = new WPI_VictorSPX(id);
        invert = victorSPX.getInverted();

        victorSPX.configFactoryDefault();
        victorSPX.setInverted(invert);
        victorSPX.stopMotor();
    
        return victorSPX;
      }


    private VictorSP makeVictorSP(int id , boolean invert) {
        VictorSP victorSP = new VictorSP(id);
        invert = victorSP.getInverted();
        
        victorSP.setInverted(invert);
        victorSP.stopMotor();

        return victorSP;
    }


    /**
     * Drive Outputs Using Smartdashboard
     */
    public void driveOutput(){
    
    }


    
    public void driveShift(boolean isShifted) {

        shifter.set(isShifted);

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
        chassis.arcadeDrive(speed, rotation_speed*sensivity);
    }

    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn){
        chassis.curvatureDrive(speed, rotation, isQuickTurn);
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
        chassis.feed();
    }

    /**
     * Fully automated drive with PID
     * @param speed
     * @param rotation_angle
     */
    public void PIDDrive(double speed, double rotation_angle){
        double drivePID = PID.calculate(Sensors.getGyroAngle(), rotation_angle);
        drivePID = MathUtil.clamp(drivePID, -1, 1);
        chassis.curvatureDrive(speed, drivePID, false);
    }

    public void resetPID(){
        PID.reset();
    }

    /**
     * Resets everything resettable
     */
    public void reset(){
        resetPID();
        Sensors.gyroReset();
    
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
        chassis.tankDrive(0, 0);
    }
}
