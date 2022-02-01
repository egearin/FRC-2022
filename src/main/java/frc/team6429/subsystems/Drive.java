// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team6429.robot.Constants;
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

    //Encoders
    public Encoder driveLeftEnc;
    public Encoder driveRightEnc;

    //Sensors 
    public PigeonIMU pigeon;
    private CANCoder driveLeftCANcoder;
    private CANCoder driveRightCANcoder;

    //Master VictorSPX
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
        rightMotor = new MotorControllerGroup(driveROne, driveRTwo);

        chassis = new DifferentialDrive(leftMotor, rightMotor);

        pigeon = new PigeonIMU(Constants.pigeonID);

        driveLeftCANcoder = new CANCoder(Constants.driveLeftCANcoderID);
        driveRightCANcoder = new CANCoder(Constants.driveRightCANcoderID);
        
        driveLeftEnc = new Encoder(Constants.driveLeftEncPort1, Constants.driveLeftEncPort2);
        driveLeftEnc.setSamplesToAverage(10);
        driveLeftEnc.setDistancePerPulse((1/Constants.EncoderCPR)* Constants.wheelPerimeter);
        
        driveRightEnc = new Encoder(Constants.driveRightEncPort1, Constants.driveRightEncPort2);
        driveRightEnc.setSamplesToAverage(10);
        driveRightEnc.setDistancePerPulse((1/Constants.EncoderCPR)* Constants.wheelPerimeter);
    
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

    public void constructOdometry(){
        constructOdometry(getFusedGyroRotation2D());
    }

    public void constructOdometry(Rotation2d gyroRotation){
        odometry = new DifferentialDriveOdometry(gyroRotation);
    }

    public void constructOdometry(Rotation2d gyroRotation, Pose2d startingPos){
        odometry = new DifferentialDriveOdometry(gyroRotation, startingPos);
    }

    public void updateOdometry(){
        updateOdometry(getFusedGyroRotation2D(), driveLeftEnc.getDistance(), driveRightEnc.getDistance());
    }

    /**
     * 
     * @param gyroRotation
     * @param leftDistance !IN METERS!
     * @param rightDistance !IN METERS!
     */
    public void updateOdometry(Rotation2d gyroRotation, double leftDistance, double rightDistance){
        odometry.update(gyroRotation, leftDistance, rightDistance);
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }
    
    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(driveLeftEnc.getRate(), driveRightEnc.getRate());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoder();
        odometry.resetPosition(pose, getFusedGyroRotation2D());
    }

    public double getSpeed(){
        return (driveLeftEnc.getRate() + driveRightEnc.getRate()) / 2; 
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
        return Rotation2d.fromDegrees(Math.IEEEremainder(getYawAngle(), 360.0));
    }

    public Rotation2d getFusedGyroRotation2D(){
        return Rotation2d.fromDegrees(Math.IEEEremainder(getGyroAngle(), 360.0));
    }

    public void resetYawAngle(){
        pigeon.setYaw(0);
    }

    public void resetEncoder(){
        driveLeftEnc.reset();
        driveRightEnc.reset();
    }

    public void resetSensors(){
        gyroReset();
        resetEncoder();
    }

    public boolean isPigeonReady(){
        PigeonState state = pigeon.getState();
        return state == PigeonState.Ready;
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
        chassis.feed();
    }

    /**
     * Fully automated drive with PID
     * @param speed
     * @param rotation_angle
     */
    public void PIDDrive(double speed, double rotation_angle){
        double drivePID = PID.calculate(getGyroAngle(), rotation_angle);
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
        chassis.tankDrive(0, 0);
    }
}
