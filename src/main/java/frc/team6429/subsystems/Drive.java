// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj.Compressor;

import frc.team6429.robot.Constants;
import frc.team6429.util.Sensors;
import frc.team6429.util.Utils;

/** 
 * Robot drive subsystem class. 
 * Includes PID & profiled PID controller, simple motor feedforward and differential drive odometry. 
 */
public class Drive implements DimensionalDrive{

    private static Drive mInstance = new Drive();

    public static Drive getInstance(){
        return mInstance;
    }

    //----------Setup----------\\
    //TalonFX
    public WPI_TalonFX driveLeftMotor;
    public WPI_TalonFX driveRightMotor;
    
    //PID 
    public PIDController PID;
    public PIDController leftController;
    public PIDController rightController;

    //ProfiledPID
    public Constraints constraints;
    public ProfiledPIDController profiledPID;

    //Trapezoid
    public TrapezoidProfile trapezoidProfile;

    //Feedforward
    public SimpleMotorFeedforward simpleMotorFF;

    //Sensors 
    public PigeonIMU pigeon;
    public CANCoder leftCANcoder;
    public CANCoder rightCANcoder;

    //Solenoid
    public Solenoid shifter;
    public Solenoid pto;

    //Master
    public MotorControllerGroup leftMotor;
    public MotorControllerGroup rightMotor;

    public DifferentialDrive chassis;
    public TalonFXConfiguration config;
    public AbsoluteSensorRange sensorRange;
    
    //Odometry
    public DifferentialDriveOdometry odometry;
    public RamseteManager ramseteManager;

    public Translation2d indexerOnCheckpoint;
    public Translation2d indexerOffCheckpoint;
    public Translation2d dumperOnCheckpoint;
    public Translation2d dumperOffCheckpoint;

    //Motor Setup
    public NeutralMode neutralModeBrake;
    public NeutralMode neutralModeCoast;
    public NeutralMode neutralModeEEPROM;

    //Other Subsystems
    public Sensors mSensors;

    //Other
    public Timer timer;

    //Constants
    double turnError;
    double turnIntegral;
    double turnDerivative;
    double turnPrevError;
    double turnRes;
    double rotation;
    double minMax;
    double distanceError;
    double prevError;
    double distanceIntegral;
    double distanceDerv;
    double resDist;
    double kP;
    double kI;
    double kD;
    double previousVelocity;
    double previousTime;
    double acceleration;
    
    /**
     * Drive Initialization
     */
    private Drive(){
        driveLeftMotor = Utils.makeTalonFX
        (Constants.driveLeftMotorID, false);
        driveRightMotor = Utils.makeTalonFX
        (Constants.driveRightMotorID, true);
        leftMotor = new MotorControllerGroup(driveLeftMotor);
        rightMotor = new MotorControllerGroup(driveRightMotor);
        chassis = new DifferentialDrive(leftMotor, rightMotor);
        config = new TalonFXConfiguration();
        driveLeftMotor.configSelectedFeedbackSensor
        (FeedbackDevice.IntegratedSensor);
        driveRightMotor.configSelectedFeedbackSensor
        (FeedbackDevice.IntegratedSensor);
        driveRightMotor.setSensorPhase(false);
        driveLeftMotor.setSensorPhase(false);
        driveRightMotor.setSelectedSensorPosition(0);
        driveLeftMotor.setSelectedSensorPosition(0);
        constraints = new Constraints
        (Constants.kMaxSpeed, Constants.kMaxAcceleration);
        PID = new PIDController
        (Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);
        profiledPID = new ProfiledPIDController
        (Constants.kDriveP, Constants.kDriveI, Constants.kDriveD, constraints);
        simpleMotorFF = new SimpleMotorFeedforward
        (Constants.kDriveS, Constants.kDriveV);
        simpleMotorFF = new SimpleMotorFeedforward
        (Constants.kDriveS, Constants.kDriveV, Constants.kDriveA);
        leftController = new PIDController
        (Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);
        rightController = new PIDController
        (Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);
        neutralModeBrake = NeutralMode.Brake;
        neutralModeCoast = NeutralMode.Coast;
        neutralModeEEPROM = NeutralMode.EEPROMSetting;
        setNaturalMode(neutralModeBrake);
        pigeon = new PigeonIMU
        (Constants.pigeonID);
        shifter = new Solenoid
        (Constants.phID, PneumaticsModuleType.REVPH, Constants.shifterChannel);
        shifter.setPulseDuration
        (Constants.shifterPulseDuration);
        pto = new Solenoid
        (Constants.phID, PneumaticsModuleType.REVPH, 11);
        pto.setPulseDuration
        (Constants.ptoPulseDuration);
        leftCANcoder = new CANCoder
        (Constants.leftCANcoderID);
        leftCANcoder.configFeedbackCoefficient
        (Constants.wheelPerimeter * Constants.degreeCoefficientCANcoder * 
        1/360, "meter", SensorTimeBase.PerSecond);
        rightCANcoder = new CANCoder
        (Constants.rightCANcoderID);
        rightCANcoder.configFeedbackCoefficient
        (Constants.wheelPerimeter * Constants.degreeCoefficientCANcoder * 
        1/360, "meter", SensorTimeBase.PerSecond);
        timer = new Timer();
        /*pto.close();
        shifter.close();
        driveLeftMotor = new WPI_TalonFX(Constants.driveLeftMotorID);
        driveRightMotor = new WPI_TalonFX(Constants.driveRightMotorID);
        driveLeftMotor.setInverted(true);
        driveRightMotor.setInverted(false);
        
        driveLeftMotor.configFactoryDefault();
        driveRightMotor.configFactoryDefault();
        driveLeftMotor.configAllSettings(config);
        driveRightMotor.configAllSettings(config);*/
    }

    public void createRamseteManager(Trajectory trajectory){
        if (ramseteManager == null){
            // Paste this variable in
            /*RamseteController disabledRamsete = new RamseteController() {
                @Override
                public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
                        double angularVelocityRefRadiansPerSecond) {
                    return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
                }
            };*/

            ramseteManager = new RamseteManager(trajectory,
                                            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),//disabledRamsete
                                            new SimpleMotorFeedforward(Constants.kDriveS, Constants.kDriveV, Constants.kDriveA),
                                            Constants.kDriveKinematics,
                                            new PIDController(Constants.kTrajectoryP, 0, 0),//leftController
                                            new PIDController(Constants.kTrajectoryP, 0, 0));//rightController
            // Reset odometry to the starting pose of the trajectory.
            ramseteManager.initialize();
        }
        else{
            DriverStation.reportError("Ramsete Manager is already assigned!", false);
        }
    }

    public void resetRamsete(){
        ramseteManager = null;
    }

    /**
     * Follow Trajectory
     * @return false if not finished true if finished
     */
    public boolean followTrajectory(){
        if (!ramseteManager.isFinished()){
            double[] motorVolts = ramseteManager.calculate(getPose(), getWheelSpeeds());
            tankDriveVolts(motorVolts[0], motorVolts[1]);
            return false;
        }
        else{
            ramseteManager.end();
            System.out.println("Dereference the ramsete");
            ramseteManager = null; // dereference object for rereference
            return true;
        }
    }
    
    /**
     * Construct differential drive odometry
     */
    public void constructOdometry(){
        constructOdometry(getFusedGyroRotation2D());
    }

    public void constructOdometry(Rotation2d gyroRotation){
        odometry = new DifferentialDriveOdometry(gyroRotation);
    }

    public void constructOdometry(Rotation2d gyroRotation, Pose2d startingPos){
        odometry = new DifferentialDriveOdometry(gyroRotation, startingPos);
    }

    /**
     * Update differential drive odometry
     */
    public void updateOdometry(){
        updateOdometry(getFusedGyroRotation2D(), 
        leftCANcoder.getPosition(), rightCANcoder.getPosition());
    }

    /**
     * @param gyroRotation
     * @param leftDistance in meters
     * @param rightDistance in meters
     */
    public void updateOdometry(Rotation2d gyroRotation, double leftDistance, double rightDistance){
        odometry.update(gyroRotation, leftDistance, rightDistance);
    }

    /**
     * Gets robot pose
     * @return
     */
    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }
    
    /**
     * Returns the current wheel speeds of the robot.
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(leftCANcoder.getVelocity(), 
        rightCANcoder.getVelocity());
    }

    /**
     * Resets the odometry to the specified pose.
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        mSensors.resetCANcoder();
        odometry.resetPosition(pose, getFusedGyroRotation2D());
    }

    /**
     * Gets average speed
     * @return
     */
    public double getSpeed(){
        return (leftCANcoder.getVelocity() + 
        rightCANcoder.getVelocity()) / 2; 
    }

    /**
     * Get Gyro Angle From Pigeon
     * @return gyro_angle
     */
    public double getGyroAngle(){
        return pigeon.getFusedHeading();
    }

    /**
     * Only get yaw angle
     */
    public double getYawAngle(){
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[0];
    }

    public Rotation2d getYawGyroRotation2D(){
        return Rotation2d.fromDegrees
        (Math.IEEEremainder(getYawAngle(), 360.0));
    }

    public Rotation2d getFusedGyroRotation2D(){
        return Rotation2d.fromDegrees
        (Math.IEEEremainder(getGyroAngle(), 360.0));
    }

    /**
     * Drive Outputs Using Smartdashboard
     */
    public void driveOutput(){
    
    }

    /**
     * Robot Drive Using Shifter
     * @param isShifted
     */
    public void driveShift(boolean isShifted){
        shifter.set(isShifted);
        SmartDashboard.putBoolean("Shifted:", isShifted );
        
        if (isShifted == true) {
            SmartDashboard.putNumber("Shift", 2);
        }

        else {
            SmartDashboard.putNumber("Shift", 1);
        }
        
    }
    /**
     * Robot Drive Using Shifter Status: Speed
     */
    public void driveShiftOne(){
        shifter.set(false);
    }

    /**
     * Robot Drive Using Shifter Status: Torque
     */
    public void driveShiftTwo(){
        shifter.set(true);
    }

    /**
     * Robot Using Power Take-Off
     * @param shift
     */
    public void powerTakeOff(boolean shift){
        pto.set(shift);
    }

    /**
     * Power Take-Off State One
     */
    public void ptoShiftOne(){
        pto.set(true);
    }
    
    /**
     * Power Take-Off State Two
     */
    public void ptoShiftTwo(){
        pto.set(false);
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

    /**
     * Robot Drive Using Curvature Drive
     * @param speed
     * @param rotation
     * @param isQuickTurn
     */
    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn){
        chassis.curvatureDrive(speed, rotation, isQuickTurn);
    }
    
    /**
    * Controls the left and right sides of the drive directly with voltages.
    *
    * @param leftVolts  the commanded left output
    * @param rightVolts the commanded right output
    */
    public void tankDriveVolts(double leftVolts, double rightVolts){
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
        double drivePID = PID.calculate(mSensors.getGyroAngle(), rotation_angle);
        drivePID = MathUtil.clamp(drivePID, -1, 1);
        chassis.curvatureDrive(speed, drivePID, false);
    }

    /**
     * Sets the goal for ProfiledPIDController
     * @param position
     * @param velocity
     */
    public void setGoal(double position, double velocity){
        profiledPID.setGoal(new State(position, velocity));
    }

    /**
     * Resets the setpoint of profiledPID
     */
    public void resetSetpoint(){
        profiledPID.reset(profiledPID.getSetpoint());
    }

    /**
     * Motor Drive using profiledPID and simpleMotorFeedforward 
     * @param wanted_angle   
     */
    public void sexyMotorDrive(double wanted_angle){
        double previousVelocity = 0;
        double previousTime;
        double acceleration;

        previousTime = Timer.getFPGATimestamp();
        acceleration = (profiledPID.getSetpoint().velocity - previousVelocity) / 
        (timer.get() - previousTime);
        
        double drivePID = profiledPID.calculate
        (mSensors.getGyroAngle(), wanted_angle);
        drivePID = MathUtil.clamp(drivePID, -1, 1);
        double driveFF = simpleMotorFF.calculate
        (mSensors.getSpeed(), acceleration);
        driveFF = MathUtil.clamp(driveFF, -1, 1);
        double voltage = driveFF + drivePID;
        
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
        
        previousVelocity = profiledPID.getSetpoint().velocity;
        
    }

    /**
     * Resets PID
     */
    public void resetPID(){
        PID.reset();
    }

    /**
     * Resets everything resettable
     */
    public void reset(){
        resetPID();
        mSensors.resetSensors();
        resetSetpoint();
    
    }

    double totalX;

    public double getTotalX(){
        return mSensors.getX();
    }

    /**
     * Distance PID Drive
     * @param maxSpeed
     * @param wantedDistance
     * @return 
     */
    public double distancePID(double maxSpeed, double wantedDistance){
        double distanceError = 0;
        double prevError = 0;
        double distanceIntegral = 0;
        double distanceDerv = 0;
        double resDist = 0;
        double kP = Constants.kDriveP;
        double kI = Constants.kDriveI;
        double kD = Constants.kDriveD;
        distanceError = wantedDistance - getTotalX();
        distanceIntegral += distanceError * .02;
        distanceDerv = (distanceError - prevError) / .02;
        resDist = kP * distanceError + kI * 
        distanceIntegral + kD * distanceDerv;
        if (wantedDistance > 0 && resDist > maxSpeed) {
            resDist = maxSpeed;
        } 
        else if (wantedDistance < 0 && resDist < -maxSpeed) {
            resDist = -maxSpeed;
        } 
        else {
        }
        
        return resDist;
    }
     
    /**
     * Simple Turn PID using rotation
     * @param desired_rotation
     * @return
     */
    public double simpleTurnPID(double desired_rotation){
        double rotation = 0;
        double kP = SmartDashboard.getNumber("Turn PID", 0.01);
        double minMax = SmartDashboard.getNumber("Min PID", 0.3);
        if (desired_rotation > 1.0) {
                rotation = kP*desired_rotation + minMax;
        }
        else if (desired_rotation < 1.0) {
                rotation = kP*desired_rotation - minMax;
        }
        return rotation;
    }

    /**
     * Turn PID using rotation
     * @param desired_rotation
     * @return 
     */
    public double turnPID(double desired_rotation){
        double rotation = 0;
        double kP = SmartDashboard.getNumber("Turn PID", 0.1);
        double minMax = 1.9;
        if (desired_rotation > 1.0) { // to the right
                rotation = kP*desired_rotation + minMax;
        }
        else if (desired_rotation < 1.0) { // to the left
                rotation = kP*desired_rotation - minMax;
        }
        tankDriveVolts(rotation, -rotation);
        return rotation;
    }

    /**
     * Turn PID using angle
     * @param wantedAngle
     * @param currentAngle
     * @return 
     */
    public double autoTurnPID(double wantedAngle, double currentAngle){
        double turnError = 0;
        double turnIntegral = 0;
        double turnDerivative = 0;
        double turnPrevError = 0;
        double turnRes = 0;

        turnError = wantedAngle-currentAngle;
        turnIntegral += turnError*Constants.kDriveI*0.02;
        turnDerivative = turnError-turnPrevError;
        turnIntegral = Utils.applyDeadband(turnIntegral, -1, 1);

        turnRes = turnIntegral+(turnError*Constants.kDriveP)+
        (turnDerivative*Constants.kDriveD);
        turnRes = Utils.applyDeadband(turnRes, -1, 1);
        
        turnPrevError = turnError;

        return turnRes;

    }
    
    public void autoTurn(double wantedAngle){
        if(Utils.tolerance(getGyroAngle(), wantedAngle, 0.5)) {
        }
        else {
        chassis.curvatureDrive(0,autoTurnPID(wantedAngle, getGyroAngle()),false);
        }
    }

     /**
     * @param neutralMode
     */
    public void neutralMode(NeutralMode neutralMode) {
		driveLeftMotor.setNeutralMode(neutralMode);
		driveRightMotor.setNeutralMode(neutralMode);
	}

    public void setNaturalMode(NeutralMode neutralMode) {
        switch(neutralMode){
            case Brake:
                neutralMode(neutralModeBrake);
                break;
            case Coast:
                neutralMode(neutralModeCoast);
                break;
            case EEPROMSetting:
                neutralMode(neutralModeEEPROM);
                break;
        }
    }

    public void talonSet(){
        
    }

    /**
     * Stop Driving Robot
     */
    public void stopDrive(){
        chassis.tankDrive(0, 0);
    }

    public enum DimensionalDrive{
        HYPER,
        DIMENSIONAL,
        HYPERDIMENSIONAL,
        OFF;
    }

    public void setDimension(DimensionalDrive dimensional){
        switch(dimensional){
            case HYPER:
                hyperDrive();
                break;
            case DIMENSIONAL:
                dimensionalDrive();
                break;
            case HYPERDIMENSIONAL:
                hyperDimensionalDrive();
                break;
            case OFF:
                stopDrive();
                break;
        }
    }
    
    @Override
    public void hyperDrive() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void dimensionalDrive() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void hyperDimensionalDrive() {
        // TODO Auto-generated method stub
        
    }


}
